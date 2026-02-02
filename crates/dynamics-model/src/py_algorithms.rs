use dynamics_spatial::configuration::Configuration;
use dynamics_spatial::py_configuration::PyConfiguration;
use dynamics_spatial::py_configuration::PyConfigurationInput;
use numpy::PyReadonlyArrayDyn;
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

use crate::forward_kinematics::frames_forward_kinematics;
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
    q: PyConfigurationInput,
    v: Option<PyConfigurationInput>,
    a: Option<PyConfigurationInput>,
) -> PyResult<()> {
    let v = match v {
        Some(v) => Some(v.to_configuration(model.inner.nv)?),
        None => None,
    };
    let a = match a {
        Some(a) => Some(a.to_configuration(model.inner.nv)?),
        None => None,
    };
    forward_kinematics(
        &model.inner,
        &mut data.inner,
        &q.to_configuration(model.inner.nq)?,
        v.as_ref(),
        a.as_ref(),
    )
    .map_err(|e| PyValueError::new_err(format!("Forward kinematics failed: {e:?}")))?;
    Ok(())
}

#[pyfunction(name = "update_frame_placements", signature=(model, data))]
pub fn py_update_frame_placements(model: &PyModel, data: &mut PyData) {
    update_frame_placements(&model.inner, &mut data.inner);
}

#[pyfunction(name = "frames_forward_kinematics", signature=(model, data, q))]
pub fn py_frames_forward_kinematics(
    model: &PyModel,
    data: &mut PyData,
    q: PyConfigurationInput,
) -> PyResult<()> {
    let q = q.to_configuration(model.inner.nq)?;

    frames_forward_kinematics(&model.inner, &mut data.inner, &q)
        .map_err(|e| PyValueError::new_err(format!("Frames forward kinematics failed: {e:?}")))?;
    Ok(())
}

#[pyfunction(name = "forward_dynamics")]
pub fn py_inverse_dynamics(
    _py: Python,
    model: &PyModel,
    data: &mut PyData,
    q: PyConfigurationInput,
    v: PyConfigurationInput,
    a: PyConfigurationInput,
) -> PyResult<PyConfiguration> {
    let q = q.to_configuration(model.inner.nq)?;
    let v = v.to_configuration(model.inner.nv)?;
    let a = a.to_configuration(model.inner.nv)?;

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
    q: PyConfigurationInput,
    v: PyConfigurationInput,
    a: PyConfigurationInput,
) -> PyResult<PyConfiguration> {
    py_inverse_dynamics(py, model, data, q, v, a)
}
