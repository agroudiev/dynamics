//! This module contains the implementation of the forward dynamics algorithms.
//! The main algorithm implemented here is the Articulated Body Algorithm (ABA).
//! The ABA computes the joint accelerations required to achieve a given motion of the robot
//! given its configuration, velocity, and torques.

use crate::data::{Data, PyData};
use crate::model::{Model, PyModel};
use numpy::PyReadonlyArrayDyn;
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use spatial::configuration::{Configuration, ConfigurationError};

/// WIP: Computes the forward dynamics of the robot model.
///
/// # Arguments
///
/// * `model` - The robot model.
/// * `data` - The data structure that contains the joint data.
/// * `q` - The configuration of the robot.
/// * `v` - The velocity of the robot.
/// * `tau` - The joint torques.
///
/// # Returns
///
/// * `Ok(ddq)` if the forward dynamics was successful.
/// * `Err(ConfigurationError)` if there was an error.
pub fn forward_dynamics(
    model: &Model,
    _data: &mut Data,
    q: &Configuration,
    v: &Configuration,
    tau: &Configuration,
) -> Result<(), ConfigurationError> {
    // check if q is of the right size
    if q.len() != model.nq {
        return Err(ConfigurationError::InvalidSize(
            "q".to_string(),
            model.nq,
            q.len(),
        ));
    }

    // check if v is of the right size
    if v.len() != model.nv {
        return Err(ConfigurationError::InvalidSize(
            "v".to_string(),
            model.nv,
            v.len(),
        ));
    }

    // check if tau is of the right size
    if tau.len() != model.nv {
        return Err(ConfigurationError::InvalidSize(
            "tau".to_string(),
            model.nv,
            tau.len(),
        ));
    }

    unimplemented!()
}

fn _aba_forward_pass_1() {
    unimplemented!()
}

fn _aba_backward_pass() {
    unimplemented!()
}

fn _aba_forward_pass_2() {
    unimplemented!()
}

#[pyfunction(name = "forward_dynamics")]
pub fn py_forward_dynamics(
    model: &PyModel,
    data: &mut PyData,
    q: PyReadonlyArrayDyn<f64>,
    v: PyReadonlyArrayDyn<f64>,
    tau: PyReadonlyArrayDyn<f64>,
) -> PyResult<()> {
    let q = Configuration::from_pyarray(q)?;
    let v = Configuration::from_pyarray(v)?;
    let tau = Configuration::from_pyarray(tau)?;

    forward_dynamics(&model.inner, &mut data.inner, &q, &v, &tau)
        .map_err(|e| PyValueError::new_err(format!("Forward dynamics failed: {:?}", e)))?;

    Ok(())
}

// Pinocchio alias (Articulated Body Algorithm)
#[pyfunction(name = "aba")]
pub fn py_aba(
    model: &PyModel,
    data: &mut PyData,
    q: PyReadonlyArrayDyn<f64>,
    v: PyReadonlyArrayDyn<f64>,
    tau: PyReadonlyArrayDyn<f64>,
) -> PyResult<()> {
    py_forward_dynamics(model, data, q, v, tau)
}
