//! Parser for the URDF (Unified Robot Description Format) file format.

use crate::errors::ParseError;
use collider::mesh::Mesh;
use collider::shape::{Cylinder, ShapeWrapper, Sphere};
use dynamics_inertia::inertia::Inertia;
use dynamics_joint::revolute::JointModelRevolute;
use dynamics_model::frame::{Frame, FrameType};
use dynamics_model::{
    geometry_model::{GeometryModel, PyGeometryModel},
    geometry_object::GeometryObject,
    model::{Model, PyModel, WORLD_FRAME_ID},
};
use dynamics_spatial::color::Color;
use dynamics_spatial::motion::SpatialRotation;
use dynamics_spatial::se3::SE3;
use dynamics_spatial::symmetric3::Symmetric3;
use dynamics_spatial::vector3d::Vector3D;
use nalgebra::Vector3;
use pyo3::prelude::*;
use roxmltree::{Document, Node};
use std::collections::HashSet;
use std::{collections::HashMap, fs, str::FromStr};

/// Parses a URDF file and builds the corresponding [`Model`] and [`GeometryModel`].
///
/// # Arguments
///
/// * `filepath` - The path to the URDF file.
///
/// # Returns
///
/// A tuple containing the [`Model`] and [`GeometryModel`] objects if successful.
/// Returns a [`ParseError`] if there is an error during parsing.
pub fn build_models_from_urdf(filepath: &str) -> Result<(Model, GeometryModel), ParseError> {
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
    let mut coll_model = GeometryModel::new();

    // start by parsing materials
    let materials = parse_materials(&robot_node)?;

    // find root nodes (links without parents)
    let root_nodes = find_root_nodes(&robot_node)?;

    // sort root nodes in alphabetical order
    let mut root_nodes: Vec<Node> = root_nodes.into_iter().collect();
    root_nodes.sort_by_key(|n| n.attribute("name").unwrap_or(""));

    // recursively parse from the root nodes
    for node in root_nodes {
        parse_node(
            &robot_node,
            node,
            &robot_node,
            WORLD_FRAME_ID, // TODO: rename to WORLD_ID or similar
            &mut model,
            &mut geom_model,
            &mut coll_model,
            &materials,
            filepath,
        )?;
    }

    Ok((model, geom_model))
}

/// Parse the given node and call itself recursively on its children.
#[allow(clippy::too_many_arguments)] // TODO: refactor
fn parse_node(
    robot_node: &Node,
    node: Node,
    parent_node: &Node,
    parent_joint_id: usize,
    model: &mut Model,
    geom_model: &mut GeometryModel,
    coll_model: &mut GeometryModel,
    materials: &HashMap<String, Color>,
    filepath: &str,
) -> Result<(), ParseError> {
    let node_name = node.attribute("name").unwrap_or("");
    let mut new_parent_joint_id = parent_joint_id;
    // parse the current node and extract its children
    let children = match node.tag_name().name() {
        "link" => {
            parse_link(
                robot_node,
                node,
                parent_node,
                parent_joint_id,
                model,
                geom_model,
                coll_model,
                materials,
                filepath,
            )?;

            // find all joints that have this link as parent
            let mut children = Vec::new();

            for joint_node in robot_node.children().filter(|n| n.has_tag_name("joint")) {
                // if this joint:
                // - has a parent tag
                // - which is a link
                // - and this link matches the current node name
                if let Some(parent_node) = joint_node.children().find(|n| n.has_tag_name("parent"))
                    && let Ok(parent_link_name) = extract_parameter::<String>("link", &parent_node)
                    && parent_link_name == node_name
                {
                    // then add this joint node to the children to be parsed next
                    children.push(joint_node);
                }
            }
            children
        }

        // parse joints
        "joint" => {
            new_parent_joint_id =
                parse_joint(robot_node, node, parent_node, parent_joint_id, model)?;

            // add the eventual child link to the children to be parsed next
            if let Some(child_node) = node.children().find(|n| n.has_tag_name("child")) {
                // find the name of the child link
                let child_link_name = extract_parameter::<String>("link", &child_node)?;

                // find the link node in the robot node
                if let Some(link_node) = robot_node.children().find(|n| {
                    n.has_tag_name("link") && n.attribute("name").unwrap_or("") == child_link_name
                }) {
                    vec![link_node]
                } else {
                    // child link not found in the robot description
                    return Err(ParseError::UnknownLinkName(child_link_name));
                }
            } else {
                vec![]
            }
        }

        // we ignore empty lines and tags not used in simulation
        "" | "gazebo" | "transmission" => {
            vec![]
        }

        // unknown tag
        _ => {
            return Err(ParseError::UnknownTag(node.tag_name().name().to_string()));
        }
    };

    // sort the children in alphabetical order
    let mut children: Vec<Node> = children.into_iter().collect();
    children.sort_by_key(|n| n.attribute("name").unwrap_or(""));

    // recursively parse children
    for child_node in children {
        parse_node(
            robot_node,
            child_node,
            &node,
            new_parent_joint_id,
            model,
            geom_model,
            coll_model,
            materials,
            filepath,
        )?;
    }
    Ok(())
}

/// Finds the root nodes (links without parents) in the robot node.
fn find_root_nodes<'a>(robot_node: &'a Node) -> Result<HashSet<Node<'a, 'a>>, ParseError> {
    // collect all link nodes
    let mut parent_links: HashSet<Node> = robot_node
        .children()
        .filter(|n| n.has_tag_name("link"))
        .collect();

    // remove all links that are children of joints
    for joint_node in robot_node.children().filter(|n| n.has_tag_name("joint")) {
        // find the parent link
        if let Some(child_node) = joint_node.children().find(|n| n.has_tag_name("child")) {
            let child_link_name = extract_parameter::<String>("link", &child_node)?;
            // remove the parent link from the set
            parent_links.retain(|link_node| {
                let link_name = link_node.attribute("name").unwrap_or("");
                link_name != child_link_name
            });
        }
    }

    Ok(parent_links)
}

/// Parses a joint from the URDF file and adds it to the model.
fn parse_joint(
    robot_node: &Node,
    joint_node: Node,
    parent_node: &Node,
    parent_joint_id: usize,
    model: &mut Model,
) -> Result<usize, ParseError> {
    // extract the name and type of joint
    let joint_name = joint_node
        .attribute("name")
        .ok_or(ParseError::NameMissing(format!("{:#?}", joint_node)))?
        .to_string();
    let joint_type = joint_node
        .attribute("type")
        .ok_or(ParseError::MissingParameter("type".to_string()))?;

    // extract the origin of the joint
    let link_origin = parse_origin(&joint_node)?;

    let parent_frame_id = get_parent_frame(parent_node, robot_node, model)?;
    let parent_frame = &model.frames[parent_frame_id];

    let new_joint_id = match joint_type {
        // if the joint is fixed, we create a fixed frame
        "fixed" => {
            let placement = parent_frame.placement * link_origin;

            let frame = Frame::new(
                joint_name.clone(),
                parent_joint_id,
                parent_frame_id,
                placement,
                FrameType::Fixed,
                Inertia::zeros(),
            );
            let _ = model.add_frame(frame, false);
            Ok(WORLD_FRAME_ID)
        }

        // if the joint is revolute, we create a revolute joint
        "revolute" => {
            // we extract the axis of rotation
            let axis_node = joint_node
                .children()
                .find(|n| n.has_tag_name("axis"))
                .ok_or(ParseError::MissingParameter("axis".to_string()))?;
            let axis = match extract_parameter_list::<f64>("xyz", &axis_node, Some(3)) {
                Ok(xyz) => Vector3D::new(xyz[0], xyz[1], xyz[2]),
                Err(e) => return Err(e),
            };

            // we extract the limit of the joint
            let limit_node = joint_node
                .children()
                .find(|n| n.has_tag_name("limit"))
                .ok_or(ParseError::MissingParameter("limit".to_string()))?;

            // TODO: extract dynamics (damping, ...)

            let mut joint_model = JointModelRevolute::new(axis);

            // optional parameters
            if let Ok(lower) = extract_parameter::<f64>("lower", &limit_node) {
                joint_model.lower_limit = lower
            };
            if let Ok(upper) = extract_parameter::<f64>("upper", &limit_node) {
                joint_model.upper_limit = upper
            };

            // required parameters
            let effort = extract_parameter::<f64>("effort", &limit_node)?;
            joint_model.effort_limit = effort;

            let velocity = extract_parameter::<f64>("velocity", &limit_node)?;
            joint_model.velocity_limit = velocity;

            model.add_joint(
                parent_joint_id,
                Box::new(joint_model),
                link_origin,
                joint_name.clone(),
            )
        }
        _ => return Err(ParseError::UnknownJointType(joint_type.to_string())),
    }
    .map_err(ParseError::ModelError)?;

    // if the joint is not fixed, we also add a frame
    if joint_type != "fixed" {
        let frame = Frame::new(
            joint_name,
            new_joint_id,
            parent_frame_id,
            SE3::identity(),
            FrameType::Joint,
            Inertia::zeros(),
        );
        model
            .add_frame(frame, true)
            .map_err(ParseError::ModelError)?;
    }

    Ok(new_joint_id)
}

/// Parses all materials defined in the robot node.
fn parse_materials(robot_node: &Node) -> Result<HashMap<String, Color>, ParseError> {
    let mut materials = HashMap::new();
    for material_node in robot_node.children().filter(|n| n.has_tag_name("material")) {
        parse_material(material_node, &mut materials)?;
    }
    Ok(materials)
}

/// Parses a link node.
#[allow(clippy::too_many_arguments)] // TODO: refactor
fn parse_link(
    robot_node: &Node,
    node: Node,
    parent_node: &Node,
    parent_joint_id: usize,
    model: &mut Model,
    geom_model: &mut GeometryModel,
    coll_model: &mut GeometryModel,
    materials: &HashMap<String, Color>,
    filepath: &str,
) -> Result<(), ParseError> {
    let link_name = node.attribute("name").unwrap_or("").to_string();

    // parse the visual node
    if let Some(visual_node) = node.children().find(|n| n.has_tag_name("visual")) {
        let geom_obj = parse_geometry(link_name.clone(), &visual_node, materials, filepath)?;
        geom_model.add_geometry_object(geom_obj);
    } else {
        // add a default geometry object if no visual node is found
        geom_model.add_geometry_object(GeometryObject::new(
            link_name.clone(),
            WORLD_FRAME_ID,
            Box::new(Sphere::new(0.0)),
            Color::transparent(),
            SE3::identity(),
        ));
    }

    // parse the inertial node
    let (link_inertia, _inertia_origin) = parse_inertia(node, link_name.clone())?;

    // compute the placement of the link frame
    let parent_frame = &model.frames[get_parent_frame(parent_node, robot_node, model)?];
    let link_placement = parent_frame.placement * parse_origin(&node)?;

    // parse the collision node
    if let Some(collision_node) = node.children().find(|n| n.has_tag_name("visual")) {
        let geom_obj = parse_geometry(link_name.clone(), &collision_node, materials, filepath)?;
        coll_model.add_geometry_object(geom_obj);
    }

    // add a frame for the link
    let parent_frame = get_parent_frame(parent_node, robot_node, model)?;
    let frame = Frame::new(
        link_name,
        parent_joint_id,
        parent_frame,
        link_placement,
        FrameType::Body,
        link_inertia, // TODO: handle inertia origin properly
    );
    model
        .add_frame(frame, true)
        .map_err(ParseError::ModelError)?;
    Ok(())
}

/// Retrieves the parent frame index from the parent node.
fn get_parent_frame(
    parent_node: &Node,
    robot_node: &Node,
    model: &Model,
) -> Result<usize, ParseError> {
    // extract the name of the parent link
    let parent_name = parent_node
        .attribute("name")
        .ok_or(ParseError::NameMissing(format!("{:#?}", parent_node)))?;
    let robot_name = robot_node
        .attribute("name")
        .ok_or(ParseError::NameMissing(format!("{:#?}", robot_node)))?;

    // check if the parent is the world frame
    if parent_name == robot_name {
        Ok(WORLD_FRAME_ID)
    } else {
        model
            .get_frame_id(parent_name)
            .ok_or(ParseError::UnknownParent(parent_name.to_string()))
    }
}

/// Parses a material from the URDF file.
fn parse_material(node: Node, materials: &mut HashMap<String, Color>) -> Result<(), ParseError> {
    // extract name
    let material_name = node
        .attribute("name")
        .ok_or(ParseError::NameMissing(format!("{:#?}", node)))?
        .to_string();

    // extract and convert color
    let color_node = node
        .children()
        .find(|n| n.has_tag_name("color"))
        .ok_or(ParseError::MaterialWithoutColor)?;
    let rgba = extract_parameter_list::<f64>("rgba", &color_node, Some(4))?;
    let color = Color::new(rgba[0], rgba[1], rgba[2], rgba[3]);

    materials.insert(material_name, color);
    Ok(())
}

/// Parses the inertia of a link from the URDF file.
fn parse_inertia(node: Node, link_name: String) -> Result<(Inertia, SE3), ParseError> {
    if let Some(inertial_node) = node.children().find(|n| n.has_tag_name("inertial")) {
        let mass_node = inertial_node
            .children()
            .find(|n| n.has_tag_name("mass"))
            .ok_or(ParseError::InertialWithoutMass(link_name.clone()))?;
        let mass = extract_parameter::<f64>("value", &mass_node)?;

        let inertia_node = inertial_node
            .children()
            .find(|n| n.has_tag_name("inertia"))
            .ok_or(ParseError::InertialWithoutInertia(link_name.clone()))?;

        let ixx = extract_parameter::<f64>("ixx", &inertia_node)?;
        let ixy = extract_parameter::<f64>("ixy", &inertia_node)?;
        let ixz = extract_parameter::<f64>("ixz", &inertia_node)?;
        let iyy = extract_parameter::<f64>("iyy", &inertia_node)?;
        let iyz = extract_parameter::<f64>("iyz", &inertia_node)?;
        let izz = extract_parameter::<f64>("izz", &inertia_node)?;

        let origin_node = inertial_node
            .children()
            .find(|n| n.has_tag_name("origin"))
            .ok_or(ParseError::MissingParameter("origin".to_string()))?;
        let rpy = extract_parameter_list::<f64>("rpy", &origin_node, Some(3))?;
        let xyz = extract_parameter_list::<f64>("xyz", &origin_node, Some(3))?;
        let rotation = SpatialRotation::from_euler_angles(rpy[0], rpy[1], rpy[2]);
        let translation = Vector3D::new(xyz[0], xyz[1], xyz[2]);
        let inertial_origin = SE3::from_parts(translation, rotation);

        Ok((
            Inertia::new(
                mass,
                translation,
                Symmetric3::new(ixx, iyy, izz, ixy, ixz, iyz),
            ),
            inertial_origin,
        ))
    } else {
        Ok((Inertia::zeros(), SE3::identity()))
    }
}

/// Parses the geometry of a link from the URDF file.
/// Extracts the geometry shape and its parameters.
fn parse_geometry(
    link_name: String,
    visual_node: &roxmltree::Node,
    materials: &HashMap<String, Color>,
    urdf_filepath: &str,
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
    } else if let Some(mesh_node) = geometry_node.children().find(|n| n.has_tag_name("mesh")) {
        let filename = mesh_node
            .attribute("filename")
            .ok_or(ParseError::MissingParameter("filename".to_string()))?;

        let absolute_path = if filename.starts_with("package://") {
            unimplemented!("package:// URDF paths are not supported yet")
        } else if filename.starts_with("/") {
            filename.to_string()
        } else {
            // treat as relative path from .urdf file
            let urdf_path = std::path::Path::new(urdf_filepath);
            let urdf_dir = match urdf_path.parent() {
                Some(dir) => dir,
                None => {
                    return Err(ParseError::InvalidFilePath(
                        "URDF file has no parent directory".to_string(),
                    ));
                }
            };
            urdf_dir
                .join(filename)
                .to_str()
                .ok_or(ParseError::InvalidFilePath(format!(
                    "Cannot convert path to string: {}",
                    urdf_dir.join(filename).display()
                )))?
                .to_string()
        };

        // check if the file exists
        if !std::path::Path::new(&absolute_path).exists() {
            return Err(ParseError::InvalidFilePath(format!(
                "Mesh file does not exist: {}",
                absolute_path
            )));
        }

        Box::new(Mesh::new(absolute_path))
    } else {
        return Err(ParseError::GeometryWithoutShape);
    };

    // extract the origin from the visual node
    let placement = parse_origin(visual_node)?;

    // extract the material color
    let mut color = Color::new(0.0, 0.0, 0.0, 1.0);
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
                color = Color::new(rgba[0], rgba[1], rgba[2], rgba[3]);
            }
        } else if let Ok(rgba) = extract_parameter_list::<f64>("rgba", &material_node, Some(4)) {
            color = Color::new(rgba[0], rgba[1], rgba[2], rgba[3]);
        }
    }

    let geom_obj = GeometryObject::new(link_name, 0, shape, color, placement);
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
    if let Some(expected_length) = expected_length
        && vector.len() != expected_length
    {
        return Err(ParseError::InvalidParameter(name.to_string()));
    }
    Ok(vector)
}

fn parse_origin(node: &roxmltree::Node) -> Result<SE3, ParseError> {
    let isometry = if let Some(origin_node) = node.children().find(|n| n.has_tag_name("origin")) {
        let xyz = extract_parameter_list::<f64>("xyz", &origin_node, Some(3))?;
        let rotation = match extract_parameter_list::<f64>("rpy", &origin_node, Some(3)) {
            Ok(rpy) => SpatialRotation::from_euler_angles(rpy[0], rpy[1], rpy[2]),
            Err(ParseError::MissingParameter(_)) => SpatialRotation::identity(),
            Err(e) => return Err(e),
        };
        let translation = Vector3D::new(xyz[0], xyz[1], xyz[2]);

        SE3::from_parts(translation, rotation)
    } else {
        SE3::identity()
    };
    Ok(isometry)
}

/// A Python wrapper for the `build_models_from_urdf` function.
#[pyfunction(name = "build_models_from_urdf")]
pub fn py_build_models_from_urdf(filepath: &str) -> PyResult<(PyModel, PyGeometryModel)> {
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
