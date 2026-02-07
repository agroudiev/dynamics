//! Define error types for parsing URDF files.

use dynamics_model::model::ModelError;

use std::{fmt::Display, io};

#[derive(Debug)]
/// Error types that can occur while parsing an URDF file.
pub enum ParseError {
    /// IO error occurred while reading the file.
    IoError(io::Error, String),
    /// Error occurred while parsing XML.
    XmlError(roxmltree::Error),
    /// The URDF file does not contain a \<robot\> tag.
    NoRobotTag,
    /// A \<visual\> tag is present without a corresponding \<geometry\> tag.
    VisualWithoutGeometry(String),
    /// A \<geometry\> tag is present without a corresponding shape tag.
    GeometryWithoutShape(String),
    /// The given required parameter is missing in the URDF.
    MissingParameter(String),
    /// The given parameter has an invalid value.
    InvalidParameter(String),
    /// A joint, link, or material is missing a name attribute.
    NameMissing(String),
    /// A material is defined without a color.
    MaterialWithoutColor(String),
    /// An unknown joint type was encountered.
    UnknownJointType(String),
    /// An unknown tag was encountered in the URDF.
    UnknownTag(String),
    /// An error occurred while building the model
    ModelError(ModelError),
    /// A link is referenced that does not exist in the model.
    UnknownLinkName(String),
    /// The file path provided for a mesh is invalid.
    InvalidFilePath(String),
    /// An inertial tag is present without inertia data.
    InertialWithoutInertia(String),
    /// An inertial tag is present without mass data.
    InertialWithoutMass(String),
    /// A frame references a parent that does not exist.
    UnknownParent(String),
}

impl Display for ParseError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ParseError::IoError(e, path) => write!(f, "IO error: {e} (file: {path})"),
            ParseError::XmlError(e) => write!(f, "XML parsing error: {e}"),
            ParseError::NoRobotTag => write!(f, "No <robot> tag found in URDF."),
            ParseError::VisualWithoutGeometry(visual) => {
                write!(
                    f,
                    "the <visual> tag named '{visual}' does not have a corresponding <geometry> tag."
                )
            }
            ParseError::GeometryWithoutShape(geometry) => {
                write!(
                    f,
                    "the <geometry> tag named '{geometry}' does not have a corresponding shape tag."
                )
            }
            ParseError::MissingParameter(param) => {
                write!(f, "Missing required parameter: {param}")
            }
            ParseError::InvalidParameter(param) => {
                write!(f, "Invalid value for parameter: {param}")
            }
            ParseError::NameMissing(entity) => write!(f, "Missing name attribute for {entity}."),
            ParseError::MaterialWithoutColor(material) => {
                write!(
                    f,
                    "the <material> tag named '{material}' does not have a corresponding <color> tag."
                )
            }
            ParseError::UnknownJointType(joint_type) => {
                write!(f, "unknown joint type ({joint_type}).")
            }
            ParseError::UnknownTag(tag) => write!(f, "Unknown tag encountered: {tag}"),
            ParseError::ModelError(e) => write!(f, "Model error: {e}"),
            ParseError::UnknownLinkName(name) => write!(f, "Unknown link name: {name}"),
            ParseError::InvalidFilePath(path) => write!(f, "Invalid file path: {path}"),
            ParseError::InertialWithoutInertia(link) => {
                write!(f, "<inertial> tag in link '{link}' missing inertia data.")
            }
            ParseError::InertialWithoutMass(link) => {
                write!(f, "<inertial> tag in link '{link}' missing mass data.")
            }
            ParseError::UnknownParent(parent) => {
                write!(f, "Frame references unknown parent: {parent}")
            }
        }
    }
}
