//! This file provides a parser for the URDF (Unified Robot Description Format) files.

use crate::errors::ParseError;
use model::model::Model;
use roxmltree::Document;
use std::fs;

pub fn build_model_from_urdf(filepath: &str) -> Result<Model, ParseError> {
    let contents = fs::read_to_string(filepath).map_err(ParseError::IoError)?;
    let _doc = Document::parse(&contents).map_err(ParseError::XmlError)?;

    let model = Model::new_empty();
    Ok(model)
}
