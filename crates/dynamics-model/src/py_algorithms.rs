use dynamics_spatial::py_configuration::PyConfiguration;
use dynamics_spatial::py_configuration::PyConfigurationInput;
use dynamics_spatial::py_force::PySpatialForce;
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

use crate::forward_dynamics::ABAConvention;
use crate::forward_kinematics::frames_forward_kinematics;
use crate::inverse_dynamics::inverse_dynamics;
use crate::neutral::neutral;
use crate::{
    forward_dynamics::forward_dynamics,
    forward_kinematics::{forward_kinematics, update_frame_placements},
    py_data::PyData,
    py_model::PyModel,
};

/// Computes the neutral configuration of a model.
///
/// This function iterates through each joint model in the given model, retrieves its neutral configuration, and assembles these into a single `Configuration` object that represents the neutral configuration of the entire model.
#[pyfunction(name = "neutral")]
pub fn py_neutral(model: &mut PyModel) -> PyResult<PyConfiguration> {
    let q = neutral(&model.inner).map_err(|e| PyErr::new::<PyValueError, _>(format!("{:?}", e)))?;
    Ok(PyConfiguration(q))
}

#[pyfunction(name = "forward_kinematics", signature=(model, data, q, v=None, a=None))]
/// Computes the forward kinematics of the robot model.
///
/// It updates the joint data and placements in the world frame.
///
/// # Arguments
///
/// * `model` - The robot model.
/// * `data` - The data structure that contains the joint data.
/// * `q` - The configuration of the robot of size `nq`.
/// * `v` - The velocity configuration of the robot of size `nv` (optional).
/// * `a` - The acceleration configuration of the robot of size `nv` (optional).
///
/// # Returns
///
/// * Updates `data.joint_data` and `data.joint_placements` in place if successful.
/// * Returns an `AlgorithmError` if there was an error.
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
/// Updates the placements of all frames in the world frame, based on the current joint placements.
///
/// This function must be called after `forward_kinematics`.
///
/// # Arguments
/// * `model` - The robot model.
/// * `data` - The data structure that contains the joint data.
///
/// # Returns
///
/// Updates `data.frame_placements` in place if successful.
pub fn py_update_frame_placements(model: &PyModel, data: &mut PyData) {
    update_frame_placements(&model.inner, &mut data.inner);
}

#[pyfunction(name = "frames_forward_kinematics", signature=(model, data, q))]
/// Computes the forward kinematics and updates the frame placements in one call.
///
/// This is a convenience function that calls `forward_kinematics` followed by `update_frame_placements`.
///
/// # Arguments
/// * `model` - The robot model.
/// * `data` - The data structure that contains the joint data.
/// * `q` - The configuration of the robot of size `nq`.
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

#[pyfunction(name = "inverse_dynamics", signature=(model, data, q, v, a, f_ext=None))]
/// Computes the inverse dynamics using the Recursive Newton-Euler Algorithm (RNEA).
///
/// # Arguments
/// * `model` - The robot model.
/// * `data` - The data structure that contains the joint data.
/// * `q` - The configuration of the robot.
/// * `v` - The velocity of the robot.
/// * `a` - The acceleration of the robot.
/// * `f_ext` - The external forces applied to the robot (optional).
///
/// # Returns
/// * `Ok(tau)` if the inverse dynamics was successful. In this case, the fields `tau`, `local_joint_placements`, `joint_velocities`, `joint_accelerations_gravity_field`, `joint_momenta` and `joint_forces` of the `data` structure are updated with the results of the algorithm.
/// * `Err(ConfigurationError)` if there was an error.
pub fn py_inverse_dynamics(
    model: &PyModel,
    data: &mut PyData,
    q: PyConfigurationInput,
    v: PyConfigurationInput,
    a: PyConfigurationInput,
    f_ext: Option<Vec<PySpatialForce>>,
) -> PyResult<PyConfiguration> {
    let q = q.to_configuration(model.inner.nq)?;
    let v = v.to_configuration(model.inner.nv)?;
    let a = a.to_configuration(model.inner.nv)?;

    let f_ext = f_ext.map(|f| f.into_iter().map(|f| f.inner).collect::<Vec<_>>());

    let tau = inverse_dynamics(&model.inner, &mut data.inner, &q, &v, &a, f_ext.as_deref())
        .map_err(|e| PyErr::new::<PyValueError, _>(format!("Error in inverse dynamics: {e}")))?;

    Ok(PyConfiguration(tau.clone()))
}

// Pinocchio alias (Recursive Newton-Euler Algorithm)
#[pyfunction(name = "rnea", signature=(model, data, q, v, a, f_ext=None))]
/// Computes the inverse dynamics using the Recursive Newton-Euler Algorithm (RNEA).
///
/// # Arguments
/// * `model` - The robot model.
/// * `data` - The data structure that contains the joint data.
/// * `q` - The configuration of the robot.
/// * `v` - The velocity of the robot.
/// * `a` - The acceleration of the robot.
/// * `f_ext` - The external forces applied to the robot (optional).
///
/// # Returns
/// * `Ok(tau)` if the inverse dynamics was successful. In this case, the fields `tau`, `local_joint_placements`, `joint_velocities`, `joint_accelerations_gravity_field`, `joint_momenta` and `joint_forces` of the `data` structure are updated with the results of the algorithm.
/// * `Err(ConfigurationError)` if there was an error.
pub fn py_rnea(
    model: &PyModel,
    data: &mut PyData,
    q: PyConfigurationInput,
    v: PyConfigurationInput,
    a: PyConfigurationInput,
    f_ext: Option<Vec<PySpatialForce>>,
) -> PyResult<PyConfiguration> {
    py_inverse_dynamics(model, data, q, v, a, f_ext)
}

#[pyfunction(name = "forward_dynamics", signature=(model, data, q, v, tau, f_ext=None, convention=None))]
/// Computes the forward dynamics of the robot model using the Articulated Body Algorithm (ABA) in the specified convention.
///
/// # Arguments
///
/// * `model` - The robot model.
/// * `data` - The data structure that contains the joint data.
/// * `q` - The configuration of the robot.
/// * `v` - The velocity of the robot.
/// * `tau` - The joint torques.
/// * `convention` - The convention to use for the ABA algorithm. If `Local`, the algorithm will be executed in the local frame of each joint. If `World`, the algorithm will be executed in the world frame.
///
/// # Returns
///
/// * `Ok(ddq)` if the forward dynamics was successful. The following fields of the `data` structure will be updated:
///     - **TODO**
/// * `Err(ConfigurationError)` if there was an error.
pub fn py_forward_dynamics(
    model: &PyModel,
    data: &mut PyData,
    q: PyConfigurationInput,
    v: PyConfigurationInput,
    tau: PyConfigurationInput,
    f_ext: Option<Vec<PySpatialForce>>,
    convention: Option<ABAConvention>,
) -> PyResult<PyConfiguration> {
    let q = q.to_configuration(model.inner.nq)?;
    let v = v.to_configuration(model.inner.nv)?;
    let tau = tau.to_configuration(model.inner.nv)?;
    let f_ext = f_ext.map(|f| f.into_iter().map(|f| f.inner).collect::<Vec<_>>());
    let convention = convention.unwrap_or(ABAConvention::Local);

    let ddq = forward_dynamics(
        &model.inner,
        &mut data.inner,
        &q,
        &v,
        &tau,
        f_ext.as_deref(),
        convention,
    )
    .map_err(|e| PyValueError::new_err(format!("Forward dynamics failed: {e:?}")))?;

    Ok(PyConfiguration(ddq.clone()))
}

#[pyfunction(name = "aba", signature=(model, data, q, v, tau, f_ext=None, convention=None))]
/// Computes the forward dynamics of the robot model using the Articulated Body Algorithm (ABA) in the specified convention.
///
/// # Arguments
///
/// * `model` - The robot model.
/// * `data` - The data structure that contains the joint data.
/// * `q` - The configuration of the robot.
/// * `v` - The velocity of the robot.
/// * `tau` - The joint torques.
/// * `convention` - The convention to use for the ABA algorithm. If `Local`, the algorithm will be executed in the local frame of each joint. If `World`, the algorithm will be executed in the world frame.
///
/// # Returns
///
/// * `Ok(ddq)` if the forward dynamics was successful. The following fields of the `data` structure will be updated:
///     - **TODO**
/// * `Err(ConfigurationError)` if there was an error.
pub fn py_aba(
    model: &PyModel,
    data: &mut PyData,
    q: PyConfigurationInput,
    v: PyConfigurationInput,
    tau: PyConfigurationInput,
    f_ext: Option<Vec<PySpatialForce>>,
    convention: Option<ABAConvention>,
) -> PyResult<PyConfiguration> {
    py_forward_dynamics(model, data, q, v, tau, f_ext, convention)
}
