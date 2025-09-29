use joint::data::JointError;
use nalgebra::DVector;
use numpy::PyReadonlyArray1;
use numpy::{ToPyArray, ndarray::Array1};
use pyo3::{Python, exceptions::PyValueError, prelude::*};

use crate::model::{Model, PyModel};

pub type Configuration = nalgebra::DVector<f64>;

pub fn configuration_from_pyarray(array: PyReadonlyArray1<f64>) -> Result<Configuration, PyErr> {
    let array = array.as_array();
    let array = match array.as_slice() {
        Some(slice) => slice,
        None => return Err(PyValueError::new_err("Failed to convert argument to slice")),
    };
    Ok(Configuration::from_row_slice(array))
}

pub enum ConfigurationError {
    InvalidSize(String, usize, usize),
    JointDataUpdateError(usize, JointError),
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
            ConfigurationError::JointDataUpdateError(id, err) => {
                write!(f, "Error updating joint data for joint {}: {:?}", id, err)
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

pub fn random_configuration(model: &Model) -> Configuration {
    let mut rng = rand::rng();
    let q = model
        .joint_models
        .values()
        .map(|joint_model| joint_model.random_configuration(&mut rng))
        .collect::<Vec<_>>();
    DVector::from_vec(q.concat())
}

#[pyfunction(name = "random_configuration")]
pub fn py_random_configuration(py: Python, model: &mut PyModel) -> Py<PyAny> {
    let q = random_configuration(&model.inner);
    Array1::from_shape_vec(model.inner.nq, q.as_slice().to_vec())
        .unwrap()
        .to_pyarray(py)
        .into_any()
        .unbind()
}
