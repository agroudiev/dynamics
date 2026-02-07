//! Errors for dynamics algorithms.
//!
//! This module defines the `AlgorithmError` enum, which represents various errors that can occur during dynamics computations.

use std::fmt::Display;

use dynamics_joint::joint_data::JointError;
use dynamics_spatial::configuration::ConfigurationError;

#[derive(Debug)]
/// Errors that can occur during dynamics computations.
pub enum AlgorithmError {
    /// An error related to the configuration of the system.
    ConfigurationError(ConfigurationError),
    /// An error related to a specific joint, identified by its name.
    JointError(String, JointError),
    /// An error indicating that an argument has an incorrect size.
    IncorrectSize {
        name: String,
        expected: usize,
        got: usize,
    },
}

impl Display for AlgorithmError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            AlgorithmError::ConfigurationError(e) => write!(f, "Configuration error: {}", e),
            AlgorithmError::JointError(joint_name, e) => {
                write!(f, "Joint '{}' error: {}", joint_name, e)
            }
            AlgorithmError::IncorrectSize {
                name,
                expected,
                got,
            } => {
                write!(
                    f,
                    "Incorrect size for argument '{}': expected {}, got {}",
                    name, expected, got
                )
            }
        }
    }
}
