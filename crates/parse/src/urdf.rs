//! This file provides a parser for the URDF (Unified Robot Description Format) files.

use crate::errors::ParseError;
use collider::shape::{Capsule, Cylinder, ShapeWrapper, Sphere};
use model::{
    geometry_model::{GeometryModel, PyGeometryModel},
    geometry_object::GeometryObject,
    model::{Model, PyModel},
};
use nalgebra::{IsometryMatrix3, Vector3};
use pyo3::prelude::*;
use roxmltree::Document;
use std::{fs, str::FromStr};

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

    for link_node in robot_node.children().filter(|n| n.has_tag_name("link")) {
        if let Some(visual_node) = link_node.children().find(|n| n.has_tag_name("visual")) {
            let link_name = link_node.attribute("name").unwrap_or("").to_string();
            let geom_obj = parse_geometry(link_name, &visual_node)?;
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
) -> Result<GeometryObject, ParseError> {
    let geometry_node = visual_node
        .children()
        .find(|n| n.has_tag_name("geometry"))
        .ok_or_else(|| ParseError::VisualWithoutGeometry)?;
    let geometry: ShapeWrapper =
        if let Some(shape_node) = geometry_node.children().find(|n| n.has_tag_name("box")) {
            let size = extract_parameter_list::<f32>("size", &shape_node)?;
            if size.len() != 3 {
                return Err(ParseError::ShapeInvalidParameter("size".to_string()));
            }
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
    let geom_obj = GeometryObject::new(link_name, 0, 0, geometry, IsometryMatrix3::identity());
    Ok(geom_obj)
}

/// Extracts a parameter from the XML node with the given name and converts it to the specified type.
/// Returns an error if the parameter is missing or cannot be parsed.
fn extract_parameter<T: FromStr>(
    name: &str,
    shape_node: &roxmltree::Node,
) -> Result<T, ParseError> {
    shape_node
        .attribute(name)
        .ok_or_else(|| ParseError::ShapeMissingParameter(name.to_string()))?
        .parse::<T>()
        .map_err(|_| ParseError::ShapeInvalidParameter(name.to_string()))
}

/// Extracts a list of parameters from the XML node with the given name and converts them to the specified type.
/// Returns an error if the parameter is missing or any value cannot be parsed.
fn extract_parameter_list<T: FromStr>(
    name: &str,
    shape_node: &roxmltree::Node,
) -> Result<Vec<T>, ParseError> {
    shape_node
        .attribute(name)
        .ok_or_else(|| ParseError::ShapeMissingParameter(name.to_string()))?
        .split_whitespace()
        .map(|s| {
            s.parse::<T>()
                .map_err(|_| ParseError::ShapeInvalidParameter(name.to_string()))
        })
        .collect()
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
