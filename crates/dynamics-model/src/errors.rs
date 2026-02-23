//! Errors for dynamics algorithms.
//!
//! This module defines the `AlgorithmError` enum, which represents various errors that can occur during dynamics computations.

use std::fmt::Display;

use dynamics_spatial::configuration::ConfigurationError;

#[derive(Debug)]
/// Errors that can occur during dynamics computations.
pub enum AlgorithmError {
    /// An error related to the configuration of the system.
    ConfigurationError(ConfigurationError),
    /// An error indicating that an argument has an incorrect size.
    IncorrectSize {
        /// The name of the argument that has an incorrect size.
        name: String,
        /// The expected size of the argument.
        expected: usize,
        /// The actual size of the argument that was provided.
        got: usize,
    },
}

impl Display for AlgorithmError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            AlgorithmError::ConfigurationError(e) => write!(f, "Configuration error: {e}"),
            AlgorithmError::IncorrectSize {
                name,
                expected,
                got,
            } => {
                write!(
                    f,
                    "Incorrect size for argument '{name}': expected {expected}, got {got}"
                )
            }
        }
    }
}
