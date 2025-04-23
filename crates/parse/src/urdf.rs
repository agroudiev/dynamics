//! Parser for the URDF (Unified Robot Description Format) file format.

use crate::errors::ParseError;
use collider::shape::{Cylinder, ShapeWrapper, Sphere};
// use joint::revolute::JointModelRevolute;
use model::{
    geometry_model::{GeometryModel, PyGeometryModel},
    geometry_object::GeometryObject,
    model::{Model, PyModel},
};
use nalgebra::{IsometryMatrix3, Translation3, Vector3, Vector4};
use pyo3::prelude::*;
use roxmltree::Document;
use std::{collections::HashMap, fs, str::FromStr};

/// Parses a URDF file and builds the corresponding `Model` and `GeometryModel`.
///
/// # Arguments
///
/// * `filepath` - The path to the URDF file.
///
/// # Returns
///
/// A tuple containing the `Model` and `GeometryModel` objects if successful.
/// Returns a `ParseError` if there is an error during parsing.
pub fn build_models_from_urdf(filepath: &str) -> Result<(Model, GeometryModel), ParseError> {
    // TODO: error if the same name is used multiple times
    let contents = fs::read_to_string(filepath).map_err(ParseError::IoError)?;
    let doc = Document::parse(&contents).map_err(ParseError::XmlError)?;

    // identify the robot node
    let robot_node = doc
        .descendants()
        .find(|n| n.tag_name().name() == "robot")
        .ok_or(ParseError::NoRobotTag)?;
    let robot_name = robot_node.attribute("name").unwrap_or("").to_string();

    // elements will sequentially be added to the models
    let mut model = Model::new(robot_name);
    let mut geom_model = GeometryModel::new();
    let mut materials: HashMap<&str, Vector4<f64>> = HashMap::new();

    let priority_order = ["material", "link", "joint"];
    let priority_map: HashMap<&str, usize> = priority_order
        .iter()
        .enumerate()
        .map(|(i, &name)| (name, i))
        .collect();

    let mut nodes = robot_node.children().collect::<Vec<_>>();
    nodes.sort_by_key(|n| priority_map.get(n.tag_name().name()).unwrap_or(&usize::MAX));

    for main_node in nodes {
        match main_node.tag_name().name() {
            // materials not attached to a link
            "material" => {
                // extract name
                let material_name = main_node.attribute("name").ok_or(ParseError::NameMissing)?;

                // extract and convert color
                let color_node = main_node
                    .children()
                    .find(|n| n.has_tag_name("color"))
                    .ok_or(ParseError::MaterialWithoutColor)?;
                let rgba = extract_parameter_list::<f64>("rgba", &color_node, Some(4))?;
                let color = Vector4::new(rgba[0], rgba[1], rgba[2], rgba[3]);

                materials.insert(material_name, color);
            }
            "link" => {
                // parse the visual node
                if let Some(visual_node) = main_node.children().find(|n| n.has_tag_name("visual")) {
                    let link_name = main_node.attribute("name").unwrap_or("").to_string();
                    let geom_obj = parse_geometry(link_name, &visual_node, &materials)?;
                    geom_model.add_geometry_object(geom_obj);
                }
            }
            // parse joints and frames
            "joint" => {
                let joint_node = main_node;

                // extract the name and type of joint
                let joint_name = joint_node
                    .attribute("name")
                    .ok_or(ParseError::NameMissing)?;
                let joint_type = joint_node
                    .attribute("type")
                    .ok_or(ParseError::MissingParameter("type".to_string()))?;

                // extract the origin of the joint if any
                let link_origin = parse_origin(&joint_node)?;

                // find the parent
                let parent_node = joint_node
                    .descendants()
                    .find(|n| n.has_tag_name("parent"))
                    .ok_or(ParseError::MissingParameter("parent".to_string()))?;
                let parent_link_name = extract_parameter::<String>("link", &parent_node)?;

                // find the child
                let child_node = joint_node
                    .descendants()
                    .find(|n| n.has_tag_name("child"))
                    .ok_or(ParseError::MissingParameter("child".to_string()))?;
                let child_link_name = extract_parameter::<String>("link", &child_node)?;

                match joint_type {
                    // fixed joints are treated as frames
                    "fixed" => {
                        // we extract the parent object to get info about its parent frame
                        let parent_object = match geom_model.indices.get(&parent_link_name) {
                            Some(parent_id) => geom_model.models.get(parent_id).unwrap(),
                            None => {
                                return Err(ParseError::MissingParameter(
                                    "parent link not found".to_string(),
                                ));
                            }
                        };

                        match (parent_object.parent_joint, parent_object.parent_frame) {
                            // if the parent is attached to a frame
                            (0, _) => {
                                // we create a new frame for the link
                                let frame_id = model.add_frame(
                                    link_origin,
                                    joint_name.to_string(),
                                    parent_object.parent_frame,
                                );

                                // we retrieve the child object to change its parent frame
                                let child_object = match geom_model.indices.get(&child_link_name) {
                                    Some(child_id) => geom_model.models.get_mut(child_id).unwrap(),
                                    None => {
                                        return Err(ParseError::MissingParameter(format!(
                                            "child link '{child_link_name}' not found"
                                        )));
                                    }
                                };
                                child_object.parent_frame = frame_id;
                            }
                            // if the parent is attached to a joint
                            (_, 0) => {
                                unimplemented!()
                            }
                            // both are set
                            (_, _) => {
                                panic!("parent joint and frame are both set")
                            }
                        }
                    }
                    "revolute" => {
                        // let axis = match extract_parameter_list::<f64>("axis", &joint_node, Some(3)) {
                        //     Ok(axis) => Vector3::new(axis[0], axis[1], axis[2]),
                        //     // default value if axis is not specified
                        //     Err(ParseError::MissingParameter(_)) => Vector3::new(1.0, 0.0, 0.0),
                        //     // if the axis is specified but invalid
                        //     Err(e) => return Err(e),
                        // };
                        // let joint_model = JointModelRevolute { axis };
                        // let _joint_id =
                        //     model.add_joint(0, Box::new(joint_model), placement, joint_name.to_string());
                        // // TODO: handle parent-child joint relationship
                    }
                    _ => return Err(ParseError::UnknownJointType(joint_type.to_string())),
                }
            }
            // we ignore empty lines and such
            "" => {}
            // unknown tag
            _ => {
                return Err(ParseError::UnknownTag(
                    main_node.tag_name().name().to_string(),
                ));
            }
        }
    }

    Ok((model, geom_model))
}

/// Parses the geometry of a link from the URDF file.
/// Extracts the geometry shape and its parameters.
fn parse_geometry(
    link_name: String,
    visual_node: &roxmltree::Node,
    materials: &HashMap<&str, Vector4<f64>>,
    // parent_frame_id: Option<usize>,
) -> Result<GeometryObject, ParseError> {
    let geometry_node = visual_node
        .children()
        .find(|n| n.has_tag_name("geometry"))
        .ok_or(ParseError::VisualWithoutGeometry)?;

    // extract the shape from the geometry node
    let shape: ShapeWrapper = if let Some(shape_node) =
        geometry_node.children().find(|n| n.has_tag_name("box"))
    {
        let size = extract_parameter_list::<f32>("size", &shape_node, Some(3))?;
        let half_extents = Vector3::new(size[0] / 2.0, size[1] / 2.0, size[2] / 2.0);
        Box::new(collider::shape::Cuboid::new(half_extents))
    } else if let Some(shape_node) = geometry_node
        .children()
        .find(|n| n.has_tag_name("cylinder"))
    {
        let radius = extract_parameter::<f32>("radius", &shape_node)?;
        let length = extract_parameter::<f32>("length", &shape_node)?;
        Box::new(Cylinder::new(radius, length / 2.0))
    } else if let Some(shape_node) = geometry_node.children().find(|n| n.has_tag_name("sphere")) {
        let radius = extract_parameter::<f32>("radius", &shape_node)?;
        Box::new(Sphere::new(radius))
    } else {
        return Err(ParseError::GeometryWithoutShape);
    };

    // extract the origin from the visual node
    let placement = parse_origin(visual_node)?;

    // extract the material color
    let mut color = Vector4::new(0.0, 0.0, 0.0, 1.0);
    // if there is a material node
    if let Some(material_node) = visual_node.children().find(|n| n.has_tag_name("material")) {
        // if this material node has a name
        if let Some(material_name) = material_node.attribute("name") {
            // if this material was already defined in the robot node
            if let Some(material_color) = materials.get(material_name) {
                color = *material_color;
            }
            // else, check if it has a color node
            else if let Ok(rgba) = extract_parameter_list::<f64>("rgba", &material_node, Some(4))
            {
                color = Vector4::new(rgba[0], rgba[1], rgba[2], rgba[3]);
            }
        } else if let Ok(rgba) = extract_parameter_list::<f64>("rgba", &material_node, Some(4)) {
            color = Vector4::new(rgba[0], rgba[1], rgba[2], rgba[3]);
        }
    }

    let geom_obj = GeometryObject::new(
        link_name, 0, 0, // parent_frame_id.unwrap_or_default(),
        shape, color, placement,
    );
    Ok(geom_obj)
}

/// Extracts a parameter from the XML node with the given name and converts it to the specified type.
/// Returns an error if the parameter is missing or cannot be parsed.
fn extract_parameter<T: FromStr>(name: &str, node: &roxmltree::Node) -> Result<T, ParseError> {
    node.attribute(name)
        .ok_or_else(|| ParseError::MissingParameter(name.to_string()))?
        .parse::<T>()
        .map_err(|_| ParseError::InvalidParameter(name.to_string()))
}

/// Extracts a list of parameters from the XML node with the given name and converts them to the specified type.
/// Returns an error if the parameter is missing or any value cannot be parsed.
fn extract_parameter_list<T: FromStr>(
    name: &str,
    node: &roxmltree::Node,
    expected_length: Option<usize>,
) -> Result<Vec<T>, ParseError> {
    let vector = node
        .attribute(name)
        .ok_or_else(|| ParseError::MissingParameter(name.to_string()))?
        .split_whitespace()
        .map(|s| {
            s.parse::<T>()
                .map_err(|_| ParseError::InvalidParameter(name.to_string()))
        })
        .collect::<Result<Vec<T>, ParseError>>()?;
    if let Some(expected_length) = expected_length {
        if vector.len() != expected_length {
            return Err(ParseError::InvalidParameter(name.to_string()));
        }
    }
    Ok(vector)
}

fn parse_origin(node: &roxmltree::Node) -> Result<IsometryMatrix3<f64>, ParseError> {
    let isometry = if let Some(origin_node) = node.children().find(|n| n.has_tag_name("origin")) {
        let xyz = extract_parameter_list::<f64>("xyz", &origin_node, Some(3))?;
        let rotation = match extract_parameter_list::<f64>("rpy", &origin_node, Some(3)) {
            Ok(rpy) => nalgebra::UnitQuaternion::from_euler_angles(rpy[0], rpy[1], rpy[2]),
            Err(ParseError::MissingParameter(_)) => nalgebra::UnitQuaternion::identity(),
            Err(e) => return Err(e),
        };
        let translation = Translation3::new(xyz[0], xyz[1], xyz[2]);

        IsometryMatrix3::from_parts(translation, rotation.to_rotation_matrix())
    } else {
        IsometryMatrix3::identity()
    };
    Ok(isometry)
}

/// A Python wrapper for the `build_models_from_urdf` function.
#[pyfunction(name = "build_models_from_urdf")]
pub fn build_models_from_urdf_py(filepath: &str) -> PyResult<(PyModel, PyGeometryModel)> {
    match build_models_from_urdf(filepath) {
        Ok((model, geom_model)) => Ok((
            PyModel { inner: model },
            PyGeometryModel { inner: geom_model },
        )),
        Err(e) => Err(PyErr::new::<pyo3::exceptions::PyValueError, _>(format!(
            "{:?}",
            e
        ))),
    }
}
