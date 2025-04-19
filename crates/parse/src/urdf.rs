//! This file provides a parser for the URDF (Unified Robot Description Format) files.

use crate::errors::ParseError;
use collider::shape::{Capsule, Cylinder, ShapeWrapper, Sphere};
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
pub fn build_models_from_urdf(filepath: &str) -> Result<(Model, GeometryModel), ParseError> {
    let contents = fs::read_to_string(filepath).map_err(ParseError::IoError)?;
    let doc = Document::parse(&contents).map_err(ParseError::XmlError)?;

    let robot_node = doc
        .descendants()
        .find(|n| n.tag_name().name() == "robot")
        .ok_or_else(|| ParseError::NoRobotTag)?;
    let robot_name = robot_node.attribute("name").unwrap_or("").to_string();
    let model = Model::new(robot_name);
    let mut geom_model = GeometryModel::new();

    // parse the materials not attached to any geometry
    let mut materials: HashMap<&str, Vector4<f64>> = HashMap::new();
    for material_node in robot_node.children().filter(|n| n.has_tag_name("material")) {
        let material_name = material_node
            .attribute("name")
            .ok_or_else(|| ParseError::MaterialWithoutName)?;
        let color_node = material_node
            .children()
            .find(|n| n.has_tag_name("color"))
            .ok_or_else(|| ParseError::MaterialWithoutColor)?;
        let rgba = extract_parameter_list::<f64>("rgba", &color_node, Some(4))?;
        let color = Vector4::new(rgba[0], rgba[1], rgba[2], rgba[3]);
        materials.insert(material_name, color);
    }

    // parse the links and their geometries
    for link_node in robot_node.children().filter(|n| n.has_tag_name("link")) {
        if let Some(visual_node) = link_node.children().find(|n| n.has_tag_name("visual")) {
            let link_name = link_node.attribute("name").unwrap_or("").to_string();
            let geom_obj = parse_geometry(link_name, &visual_node, &materials)?;
            geom_model.add_geometry_object(geom_obj);
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
) -> Result<GeometryObject, ParseError> {
    let geometry_node = visual_node
        .children()
        .find(|n| n.has_tag_name("geometry"))
        .ok_or_else(|| ParseError::VisualWithoutGeometry)?;

    // extract the shape from the geometry node
    let shape: ShapeWrapper =
        if let Some(shape_node) = geometry_node.children().find(|n| n.has_tag_name("box")) {
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
        } else if geometry_node.children().any(|n| n.has_tag_name("sphere")) {
            let radius = extract_parameter::<f32>("radius", &geometry_node)?;
            Box::new(Sphere::new(radius))
        } else if geometry_node.children().any(|n| n.has_tag_name("capsule")) {
            let radius = extract_parameter::<f32>("radius", &geometry_node)?;
            let length = extract_parameter::<f32>("length", &geometry_node)?;
            Box::new(Capsule::new(radius, length / 2.0))
        } else {
            return Err(ParseError::GeometryWithoutShape);
        };

    // extract the origin from the visual node
    // TODO: convert the placement to a placement relative to the parent link
    let placement =
        if let Some(origin_node) = visual_node.children().find(|n| n.has_tag_name("origin")) {
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

    // extract the material color
    let mut color = Vector4::new(0.0, 0.0, 0.0, 1.0);
    // if there is a material node
    if let Some(material_node) = visual_node.children().find(|n| n.has_tag_name("material")) {
        // if this material node has a name
        if let Some(material_name) = material_node.attribute("name") {
            // if this material was already defined in the robot node
            if let Some(material_color) = materials.get(material_name) {
                color = material_color.clone();
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

    let geom_obj = GeometryObject::new(link_name, 0, 0, shape, color, placement);
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
