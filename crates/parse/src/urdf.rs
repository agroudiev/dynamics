//! This file provides a parser for the URDF (Unified Robot Description Format) files.

use crate::errors::ParseError;
use collider::shape::capsule::Capsule;
use model::{geometry_model::GeometryModel, geometry_object::GeometryObject, model::Model};
use nalgebra::IsometryMatrix3;
use roxmltree::Document;
use std::fs;

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
            let geom_obj = parse_geometry(&visual_node)?;
            let link_name = link_node.attribute("name").unwrap_or("").to_string();
            geom_model.add_geometry_object(geom_obj);
        }
    }

    Ok((model, geom_model))
}

fn parse_geometry(visual_node: &roxmltree::Node) -> Result<GeometryObject, ParseError> {
    let geom_obj = GeometryObject::new(
        String::new(),
        0,
        0,
        Box::new(Capsule::new(0.0, 0.0)),
        IsometryMatrix3::identity(),
    );
    Ok(geom_obj)
}
