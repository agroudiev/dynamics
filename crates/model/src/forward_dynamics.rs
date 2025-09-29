use crate::configuration::{Configuration, ConfigurationError};
use crate::data::{Data, PyData};
use crate::model::{Model, PyModel};
use numpy::PyReadonlyArray1;
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

/// Computes the forward dynamics of the robot model.
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
    data: &mut Data,
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

    Ok(unimplemented!())
}

fn aba_forward_pass_1() {
    unimplemented!()
}

fn aba_backward_pass() {
    unimplemented!()
}

fn aba_forward_pass_2() {
    unimplemented!()
}

#[pyfunction(name = "forward_dynamics")]
pub fn py_forward_dynamics(
    model: &PyModel,
    data: &mut PyData,
    q: PyReadonlyArray1<f64>,
    v: PyReadonlyArray1<f64>,
    tau: PyReadonlyArray1<f64>,
) -> PyResult<()> {
    // TODO: use a function to convert the PyReadonlyArray1 to a configuration
    let q = q.as_array();
    let q = match q.as_slice() {
        Some(slice) => slice,
        None => return Err(PyValueError::new_err("Failed to convert q to slice")),
    };
    let q = Configuration::from_row_slice(q);

    let v = v.as_array();
    let v = match v.as_slice() {
        Some(slice) => slice,
        None => return Err(PyValueError::new_err("Failed to convert v to slice")),
    };
    let v = Configuration::from_row_slice(v);

    let tau = tau.as_array();
    let tau = match tau.as_slice() {
        Some(slice) => slice,
        None => return Err(PyValueError::new_err("Failed to convert tau to slice")),
    };
    let tau = Configuration::from_row_slice(tau);

    forward_dynamics(&model.inner, &mut data.inner, &q, &v, &tau)
        .map_err(|e| PyValueError::new_err(format!("Forward dynamics failed: {:?}", e)))?;

    Ok(())
}
