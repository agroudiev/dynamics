use std::ops::{Add, Index, Mul};

use approx::{AbsDiffEq, RelativeEq};
use nalgebra::DVector;
use numpy::{PyReadonlyArrayDyn, ToPyArray, ndarray::Array1};
use pyo3::prelude::*;

#[derive(Clone, Debug, PartialEq)]
/// Configuration of a multi-body system, represented as a vector of joint positions.
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

    pub fn from_vec(data: Vec<f64>) -> Self {
        Configuration(DVector::from_vec(data))
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

impl AbsDiffEq for Configuration {
    type Epsilon = f64;

    fn default_epsilon() -> Self::Epsilon {
        f64::default_epsilon()
    }

    fn abs_diff_eq(&self, other: &Self, epsilon: Self::Epsilon) -> bool {
        self.0.abs_diff_eq(&other.0, epsilon)
    }
}

impl RelativeEq for Configuration {
    fn default_max_relative() -> f64 {
        f64::default_max_relative()
    }

    fn relative_eq(&self, other: &Self, epsilon: f64, max_relative: f64) -> bool {
        self.0.relative_eq(&other.0, epsilon, max_relative)
    }
}

impl Add for Configuration {
    type Output = Configuration;

    fn add(self, rhs: Self) -> Self::Output {
        Configuration(self.0 + rhs.0)
    }
}

impl Add for &Configuration {
    type Output = Configuration;

    fn add(self, rhs: Self) -> Self::Output {
        Configuration(&self.0 + &rhs.0)
    }
}

impl Mul<f64> for &Configuration {
    type Output = DVector<f64>;

    fn mul(self, rhs: f64) -> Self::Output {
        &self.0 * rhs
    }
}

#[pyclass(name = "Configuration")]
/// Python wrapper for the `Configuration` struct.
pub struct PyConfiguration(Configuration);

impl PyConfiguration {
    pub fn new(config: Configuration) -> Self {
        PyConfiguration(config)
    }

    pub fn from_pyarray(array: PyReadonlyArrayDyn<f64>) -> Result<Self, PyErr> {
        let config = Configuration::from_pyarray(array)?;
        Ok(PyConfiguration::new(config))
    }

    pub fn to_configuration(&self) -> &Configuration {
        &self.0
    }
}

#[pymethods]
impl PyConfiguration {
    pub fn __repr__(slf: PyRef<'_, Self>) -> String {
        format!("{:#?}", slf.0)
    }

    pub fn __mul__(&self, other: f64) -> PyConfiguration {
        let result = &self.0 * other;
        PyConfiguration(Configuration(result))
    }

    pub fn __add__(&self, other: &PyConfiguration) -> PyConfiguration {
        let result = &self.0 + &other.0;
        PyConfiguration(result)
    }

    pub fn to_numpy(&self, py: Python) -> Py<PyAny> {
        Array1::from_iter(self.0.0.iter().copied())
            .to_pyarray(py)
            .into_any()
            .unbind()
    }
}

/// Errors that can occur when working with configurations.
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
