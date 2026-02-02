use numpy::{PyReadonlyArray1, PyReadonlyArrayDyn, ToPyArray, ndarray::Array1};
use pyo3::{exceptions::PyValueError, prelude::*};

use crate::configuration::Configuration;

#[pyclass(name = "Configuration")]
#[derive(Debug, Clone, PartialEq)]
/// Python wrapper for the `Configuration` struct.
pub struct PyConfiguration(pub Configuration);

impl PyConfiguration {
    #[must_use]
    pub fn new(config: Configuration) -> Self {
        PyConfiguration(config)
    }

    pub fn from_pyarray(array: &PyReadonlyArrayDyn<f64>) -> Result<Self, PyErr> {
        let config = Configuration::from_pyarray(array)?;
        Ok(PyConfiguration::new(config))
    }

    #[must_use]
    pub fn to_configuration(&self) -> &Configuration {
        &self.0
    }
}

#[pymethods]
impl PyConfiguration {
    #[must_use]
    pub fn __repr__(slf: PyRef<'_, Self>) -> String {
        format!("{}", slf.0)
    }

    #[must_use]
    pub fn __mul__(&self, other: f64) -> PyConfiguration {
        let result = &self.0 * other;
        PyConfiguration(Configuration(result))
    }

    #[must_use]
    pub fn __add__(&self, other: &PyConfiguration) -> PyConfiguration {
        let result = &self.0 + &other.0;
        PyConfiguration(result)
    }

    #[must_use]
    pub fn to_numpy(&self, py: Python) -> Py<PyAny> {
        Array1::from_iter(self.0.0.iter().copied())
            .to_pyarray(py)
            .into_any()
            .unbind()
    }
}

#[derive(FromPyObject)]
pub enum PyConfigurationInput<'py> {
    Configuration(PyConfiguration),
    Array(PyReadonlyArray1<'py, f64>),
}

impl PyConfigurationInput<'_> {
    pub fn to_configuration(&self, nq: usize) -> Result<Configuration, PyErr> {
        match self {
            PyConfigurationInput::Configuration(py_config) => Ok(py_config.0.clone()), // TODO: avoid clone
            PyConfigurationInput::Array(array) => {
                let q = array.as_array();
                if q.shape() != [nq] {
                    return Err(PyValueError::new_err(format!(
                        "Invalid input size. Expected a configuration of size {}, got {:?}",
                        nq,
                        q.shape()
                    )));
                }
                let q = match q.as_slice() {
                    Some(slice) => slice,
                    None => return Err(PyValueError::new_err("Failed to convert 'q' to slice")),
                };
                let q = Configuration::from_row_slice(q);
                Ok(q)
            }
        }
    }
}
