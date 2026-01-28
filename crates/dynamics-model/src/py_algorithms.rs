use dynamics_spatial::configuration::Configuration;
use dynamics_spatial::py_configuration::PyConfiguration;
use numpy::PyReadonlyArray1;
use numpy::PyReadonlyArrayDyn;
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

use crate::inverse_dynamics::inverse_dynamics;
use crate::neutral::neutral;
use crate::{
    forward_dynamics::forward_dynamics,
    forward_kinematics::{forward_kinematics, update_frame_placements},
    py_data::PyData,
    py_model::PyModel,
};

/// Python wrapper for the `neutral` function.
#[pyfunction(name = "neutral")]
pub fn py_neutral(model: &mut PyModel) -> PyResult<PyConfiguration> {
    let q = neutral(&mut model.inner)
        .map_err(|e| PyErr::new::<pyo3::exceptions::PyValueError, _>(format!("{:?}", e)))?;
    Ok(PyConfiguration::new(q))
}

#[pyfunction(name = "forward_dynamics")]
pub fn py_forward_dynamics(
    model: &PyModel,
    data: &mut PyData,
    q: PyReadonlyArrayDyn<f64>,
    v: PyReadonlyArrayDyn<f64>,
    tau: PyReadonlyArrayDyn<f64>,
) -> PyResult<()> {
    let q = Configuration::from_pyarray(&q)?;
    let v = Configuration::from_pyarray(&v)?;
    let tau = Configuration::from_pyarray(&tau)?;

    forward_dynamics(&model.inner, &mut data.inner, &q, &v, &tau)
        .map_err(|e| PyValueError::new_err(format!("Forward dynamics failed: {e:?}")))?;

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

#[pyfunction(name = "forward_kinematics", signature=(model, data, q, v=None, a=None))]
pub fn py_forward_kinematics(
    model: &PyModel,
    data: &mut PyData,
    q: PyReadonlyArray1<f64>,
    v: Option<PyReadonlyArray1<f64>>,
    a: Option<PyReadonlyArray1<f64>>,
) -> PyResult<()> {
    let q = q.as_array();
    if q.shape() != [model.inner.nq] {
        return Err(PyValueError::new_err(format!(
            "Invalid input size. Expected a configuration of size {}, got {:?}",
            model.inner.nq,
            q.shape()
        )));
    }
    let q = match q.as_slice() {
        Some(slice) => slice,
        None => return Err(PyValueError::new_err("Failed to convert 'q' to slice")),
    };
    let q = Configuration::from_row_slice(q);

    let v = match v {
        Some(v_array) => {
            let v_array = v_array.as_array();
            if v_array.shape() != [model.inner.nv] {
                return Err(PyValueError::new_err(format!(
                    "Invalid input size. Expected a configuration of size {}, got {:?}",
                    model.inner.nv,
                    v_array.shape()
                )));
            }
            let v_slice = match v_array.as_slice() {
                Some(slice) => slice,
                None => return Err(PyValueError::new_err("Failed to convert 'v' to slice")),
            };
            Some(Configuration::from_row_slice(v_slice))
        }
        None => None,
    };

    let a = match a {
        Some(a_array) => {
            let a_array = a_array.as_array();
            if a_array.shape() != [model.inner.nv] {
                return Err(PyValueError::new_err(format!(
                    "Invalid input size. Expected a configuration of size {}, got {:?}",
                    model.inner.nv,
                    a_array.shape()
                )));
            }
            let a_slice = match a_array.as_slice() {
                Some(slice) => slice,
                None => return Err(PyValueError::new_err("Failed to convert 'a' to slice")),
            };
            Some(Configuration::from_row_slice(a_slice))
        }
        None => None,
    };

    forward_kinematics(&model.inner, &mut data.inner, &q, &v, &a)
        .map_err(|e| PyValueError::new_err(format!("Forward kinematics failed: {e:?}")))?;
    Ok(())
}

#[pyfunction(name = "update_frame_placements", signature=(model, data))]
pub fn py_update_frame_placements(model: &PyModel, data: &mut PyData) {
    update_frame_placements(&model.inner, &mut data.inner);
}

#[pyfunction(name = "forward_dynamics")]
pub fn py_inverse_dynamics(
    _py: Python,
    model: &PyModel,
    data: &mut PyData,
    q: PyReadonlyArrayDyn<f64>,
    v: PyReadonlyArrayDyn<f64>,
    a: PyReadonlyArrayDyn<f64>,
) -> PyResult<PyConfiguration> {
    let q = Configuration::from_pyarray(&q)?;
    let v = Configuration::from_pyarray(&v)?;
    let a = Configuration::from_pyarray(&a)?;

    let tau = inverse_dynamics(&model.inner, &mut data.inner, &q, &v, &a).map_err(|e| {
        PyErr::new::<pyo3::exceptions::PyValueError, _>(format!("Error in inverse dynamics: {e}"))
    })?;

    Ok(PyConfiguration::new(tau))
}

// Pinocchio alias (Recursive Newton-Euler Algorithm)
#[pyfunction(name = "rnea")]
pub fn py_rnea(
    py: Python,
    model: &PyModel,
    data: &mut PyData,
    q: PyReadonlyArrayDyn<f64>,
    v: PyReadonlyArrayDyn<f64>,
    a: PyReadonlyArrayDyn<f64>,
) -> PyResult<PyConfiguration> {
    py_inverse_dynamics(py, model, data, q, v, a)
}
