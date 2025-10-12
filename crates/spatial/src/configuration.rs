use std::ops::Index;

use nalgebra::DVector;
use numpy::PyReadonlyArrayDyn;
use pyo3::prelude::*;

#[derive(Clone, Debug, PartialEq)]
pub struct Configuration(DVector<f64>);

impl Configuration {
    pub fn zeros(size: usize) -> Self {
        Configuration(DVector::zeros(size))
    }

    pub fn ones(size: usize) -> Self {
        Configuration(DVector::from_element(size, 1.0))
    }

    pub fn len(&self) -> usize {
        self.0.len()
    }

    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }

    pub fn rows(&self, start: usize, nrows: usize) -> Configuration {
        Configuration(self.0.rows(start, nrows).into_owned())
    }

    pub fn update_rows(&mut self, start: usize, values: &Configuration) {
        assert_eq!(
            self.0.rows(start, values.len()),
            values.0,
            "Mismatched sizes when updating configuration rows."
        );
        self.0.rows_mut(start, values.len()).copy_from(&values.0);
    }

    pub fn from_row_slice(data: &[f64]) -> Self {
        Configuration(DVector::from_row_slice(data))
    }

    pub fn from_pyarray(array: PyReadonlyArrayDyn<f64>) -> Result<Configuration, PyErr> {
        let array = array.as_array();
        let flat: Vec<f64> = array.iter().copied().collect();
        Ok(Configuration::from_row_slice(&flat))
    }

    pub fn concat(configs: &[Configuration]) -> Configuration {
        let mut all_values = Vec::new();
        for config in configs {
            all_values.extend_from_slice(config.0.as_slice());
        }
        Configuration::from_row_slice(&all_values)
    }
}

impl Index<usize> for Configuration {
    type Output = f64;

    fn index(&self, index: usize) -> &Self::Output {
        &self.0[index]
    }
}

#[pyclass(name = "Configuration")]
pub struct PyConfiguration(Configuration);

impl PyConfiguration {
    pub fn new(config: Configuration) -> Self {
        PyConfiguration(config)
    }
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
