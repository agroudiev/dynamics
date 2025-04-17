use std::io;

#[derive(Debug)]
pub enum ParseError {
    IoError(io::Error),
    XmlError(roxmltree::Error),
    NoRobotTag,
}
