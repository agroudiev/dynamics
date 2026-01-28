use std::fmt::Display;

use dynamics_joint::joint_data::JointError;
use dynamics_spatial::configuration::ConfigurationError;

#[derive(Debug)]
pub enum AlgorithmError {
    ConfigurationError(ConfigurationError),
    JointError(String, JointError),
}

impl Display for AlgorithmError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            AlgorithmError::ConfigurationError(e) => write!(f, "Configuration error: {}", e),
            AlgorithmError::JointError(joint_name, e) => {
                write!(f, "Joint '{}' error: {}", joint_name, e)
            }
        }
    }
}
