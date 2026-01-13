//! Define error types for parsing URDF files.

use dynamics_model::model::ModelError;

use std::io;

#[derive(Debug)]
/// Error types that can occur while parsing an URDF file.
pub enum ParseError {
    /// IO error occurred while reading the file.
    IoError(io::Error),
    /// Error occurred while parsing XML.
    XmlError(roxmltree::Error),
    /// The URDF file does not contain a <robot> tag.
    NoRobotTag,
    /// A <visual> tag is present without a corresponding <geometry> tag.
    VisualWithoutGeometry,
    /// A <geometry> tag is present without a corresponding shape tag.
    GeometryWithoutShape,
    /// The given required parameter is missing in the URDF.
    MissingParameter(String),
    /// The given parameter has an invalid value.
    InvalidParameter(String),
    /// A joint, link, or material is missing a name attribute.
    NameMissing,
    /// A material is defined without a color.
    MaterialWithoutColor,
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
