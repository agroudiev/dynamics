use std::io;

/// Error types for parsing URDF files.
#[derive(Debug)]
pub enum ParseError {
    IoError(io::Error),
    XmlError(roxmltree::Error),
    NoRobotTag,
    VisualWithoutGeometry,
    GeometryWithoutShape,
    MissingParameter(String),
    InvalidParameter(String),
    NameMissing,
    MaterialWithoutColor,
    UnknownJointType(String),
    UnknownTag(String),
    ModelError(String),
    UnknownLinkName(String),
}
