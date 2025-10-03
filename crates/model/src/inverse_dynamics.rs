//! This module contains the implementation of the inverse dynamics algorithms.
//! The main algorithm implemented here is the Recursive Newton-Euler Algorithm (RNEA).
//! The RNEA computes the joint torques required to achieve a given motion of the robot
//! given its configuration, velocity, and acceleration.

use crate::configuration::{Configuration, ConfigurationError, configuration_from_pyarray};
use crate::data::{Data, PyData};
use crate::model::{Model, PyModel};
use numpy::PyReadonlyArrayDyn;
use pyo3::prelude::*;

/// Computes the inverse dynamics using the Recursive Newton-Euler Algorithm (RNEA).
///
/// # Arguments
/// * `model` - The robot model.
/// * `data` - The data structure that contains the joint data.
/// * `q` - The configuration of the robot.
/// * `v` - The velocity of the robot.
/// * `a` - The acceleration of the robot.
///
/// # Returns
/// * `Ok(tau)` if the inverse dynamics was successful.
/// * `Err(ConfigurationError)` if there was an error.
pub fn inverse_dynamics(
    model: &Model,
    data: &mut Data,
    q: &Configuration,
    v: &Configuration,
    a: &Configuration,
) -> Result<(), ConfigurationError> {
    unimplemented!()
}

#[pyfunction(name = "forward_dynamics")]
pub fn py_inverse_dynamics(
    model: &PyModel,
    data: &mut PyData,
    q: PyReadonlyArrayDyn<f64>,
    v: PyReadonlyArrayDyn<f64>,
    a: PyReadonlyArrayDyn<f64>,
) -> PyResult<()> {
    let q = configuration_from_pyarray(q)?;
    let v = configuration_from_pyarray(v)?;
    let a = configuration_from_pyarray(a)?;

    inverse_dynamics(&model.inner, &mut data.inner, &q, &v, &a).map_err(|e| {
        PyErr::new::<pyo3::exceptions::PyValueError, _>(format!("Error in inverse dynamics: {}", e))
    })
}

// Pinocchio alias (Recursive Newton-Euler Algorithm)
#[pyfunction(name = "rnea")]
pub fn py_rnea(
    model: &PyModel,
    data: &mut PyData,
    q: PyReadonlyArrayDyn<f64>,
    v: PyReadonlyArrayDyn<f64>,
    a: PyReadonlyArrayDyn<f64>,
) -> PyResult<()> {
    py_inverse_dynamics(model, data, q, v, a)
}
