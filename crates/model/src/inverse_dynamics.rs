//! This module contains the implementation of the inverse dynamics algorithms.
//! The main algorithm implemented here is the Recursive Newton-Euler Algorithm (RNEA).
//! The RNEA computes the joint torques required to achieve a given motion of the robot
//! given its configuration, velocity, and acceleration.

use crate::data::{Data, PyData};
use crate::model::{Model, PyModel};
use joint::joint::JointWrapper;
use nalgebra::{IsometryMatrix3, Vector6};
use numpy::ndarray::Array1;
use numpy::{PyReadonlyArrayDyn, ToPyArray};
use pyo3::prelude::*;
use spatial::configuration::{Configuration, ConfigurationError, configuration_from_pyarray};
use spatial::se3::SE3;
use std::collections::HashMap;

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
) -> Result<Configuration, ConfigurationError> {
    let mut position_transforms: HashMap<usize, SE3> = HashMap::new();
    let mut velocities: HashMap<usize, Vector6<f64>> = HashMap::new();
    let mut accelerations: HashMap<usize, Vector6<f64>> = HashMap::new();

    let mut offset = 0;
    let mut keys: Vec<_> = model.joint_models.keys().cloned().collect();
    keys.sort();
    for id in keys {
        // retrieve the joint model and the corresponding configuration
        let joint_model: Box<&JointWrapper> = Box::new(model.joint_models.get(&id).unwrap());
        let parent_id = model.joint_parents[&id];

        // extract the joint configuration, velocity and acceleration from configuration vectors
        let q_joint = q.rows(offset, joint_model.nq()).into_owned();
        let v_joint = v.rows(offset, joint_model.nq()).into_owned();
        let a_joint = a.rows(offset, joint_model.nq()).into_owned();

        // compute the transformation matrix of the joint (X_J) and axis (S_i)
        let transform = joint_model.transform(&q_joint);
        let axis = match joint_model.get_axis() {
            Some(axis_3) => axis_3.insert_rows(3, 3, 0.0),
            None => unimplemented!("implement fixed joint case"),
        };

        // local joint placement (X_T(i))
        let local_joint_placement = model.joint_placements.get(&id).unwrap();

        // local velocity
        let local_velocity = axis * v_joint;

        // compute the position, velocity and acceleration of the joint
        position_transforms.insert(id, transform * local_joint_placement);
        // velocities.insert(id, position_transforms[&parent_id] * velocities[&parent_id] + local_velocity);

        offset += joint_model.nq();
    }

    // TODO: add external forces

    let tau = nalgebra::DVector::<f64>::zeros(model.nv);

    Ok(tau)
}

#[pyfunction(name = "forward_dynamics")]
pub fn py_inverse_dynamics(
    py: Python,
    model: &PyModel,
    data: &mut PyData,
    q: PyReadonlyArrayDyn<f64>,
    v: PyReadonlyArrayDyn<f64>,
    a: PyReadonlyArrayDyn<f64>,
) -> PyResult<Py<PyAny>> {
    let q = configuration_from_pyarray(q)?;
    let v = configuration_from_pyarray(v)?;
    let a = configuration_from_pyarray(a)?;

    let tau = inverse_dynamics(&model.inner, &mut data.inner, &q, &v, &a).map_err(|e| {
        PyErr::new::<pyo3::exceptions::PyValueError, _>(format!("Error in inverse dynamics: {}", e))
    })?;

    Ok(Array1::from_vec(tau.data.as_vec().clone())
        .to_pyarray(py)
        .into_any()
        .unbind())
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
) -> PyResult<Py<PyAny>> {
    py_inverse_dynamics(py, model, data, q, v, a)
}
