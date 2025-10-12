use nalgebra::DVector;
use numpy::PyReadonlyArrayDyn;
use numpy::{ToPyArray, ndarray::Array1};
use pyo3::{Python, prelude::*};

pub type Configuration = nalgebra::DVector<f64>;

pub fn configuration_from_pyarray(array: PyReadonlyArrayDyn<f64>) -> Result<Configuration, PyErr> {
    let array = array.as_array();
    let flat: Vec<f64> = array.iter().copied().collect();
    Ok(Configuration::from_row_slice(&flat))
}

pub enum ConfigurationError {
    InvalidSize(String, usize, usize),
}

impl std::fmt::Display for ConfigurationError {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        match self {
            ConfigurationError::InvalidSize(name, expected, actual) => {
                write!(
                    f,
                    "Parameter '{}' expected configuration size {}, but got {}",
                    name, expected, actual
                )
            }
        }
    }
}

impl std::fmt::Debug for ConfigurationError {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "{}", self)
    }
}

impl std::error::Error for ConfigurationError {}
