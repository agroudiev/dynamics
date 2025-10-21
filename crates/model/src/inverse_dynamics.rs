//! This module contains the implementation of the inverse dynamics algorithms.
//! The main algorithm implemented here is the Recursive Newton-Euler Algorithm (RNEA).
//! The RNEA computes the joint torques required to achieve a given motion of the robot
//! given its configuration, velocity, and acceleration.

use crate::data::{Data, PyData};
use crate::model::{Model, PyModel};
use joint::joint::JointWrapper;
use numpy::PyReadonlyArrayDyn;
use pyo3::prelude::*;
use spatial::configuration::{Configuration, ConfigurationError, PyConfiguration};
use spatial::motion::SpatialMotion;
use spatial::vector3d::Vector3D;
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
    let mut position_transforms = HashMap::with_capacity(model.joint_models.len());
    let mut velocities = HashMap::with_capacity(model.joint_models.len());
    let mut accelerations = HashMap::with_capacity(model.joint_models.len());
    let mut forces = HashMap::with_capacity(model.joint_models.len());

    velocities.insert(0, SpatialMotion::zero());
    accelerations.insert(
        0,
        SpatialMotion::from_parts(Vector3D::zeros(), model.gravity),
    );
    forces.insert(0, SpatialMotion::zero());

    let mut offset = 0;
    let mut keys: Vec<_> = model.joint_models.keys().cloned().collect();
    keys.sort();
    for id in keys {
        // retrieve the joint model and the corresponding configuration
        let joint_model: Box<&JointWrapper> = Box::new(model.joint_models.get(&id).unwrap());
        let parent_id = model.joint_parents[&id];

        // extract the joint configuration, velocity and acceleration from configuration vectors
        let q_joint = q.rows(offset, joint_model.nq());
        let v_joint = v.rows(offset, joint_model.nq());
        let a_joint = a.rows(offset, joint_model.nq());

        // compute the transformation matrix of the joint (X_J) and axis (S_i)
        let transform = joint_model.transform(&q_joint);
        let axis = joint_model.get_axis();

        let inertia = model.inertias.get(&id).unwrap();

        // local joint placement (X_T(i))
        let local_joint_placement = model.joint_placements.get(&id).unwrap();

        // local velocity
        // TODO: use a matrix multiplication instead of a loop
        let mut local_velocity = Vec::with_capacity(joint_model.nq());
        for (i, axis_i) in axis.iter().enumerate() {
            local_velocity.push(axis_i * v_joint[i]);
        }
        let local_velocity = local_velocity
            .into_iter()
            .fold(SpatialMotion::zero(), |acc, x| acc + x);

        // local acceleration
        let mut local_acceleration = Vec::with_capacity(joint_model.nq());
        for (i, axis_i) in axis.iter().enumerate() {
            local_acceleration.push(axis_i * a_joint[i]);
        }
        let local_acceleration = local_acceleration
            .into_iter()
            .fold(SpatialMotion::zero(), |acc, x| acc + x);

        // compute the position, velocity and acceleration of the joint
        position_transforms.insert(id, (transform * local_joint_placement).action());
        velocities.insert(
            id,
            position_transforms[&parent_id] * &velocities[&parent_id] + &local_velocity,
        );
        accelerations.insert(
            id,
            position_transforms[&parent_id] * &accelerations[&parent_id]
                + local_acceleration
                + velocities[&id].cross(&local_velocity),
        );
        forces.insert(id, inertia * &accelerations[&id]);

        offset += joint_model.nq();
    }

    // TODO: add external forces

    let tau = Configuration::zeros(model.nv);

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
) -> PyResult<PyConfiguration> {
    let q = Configuration::from_pyarray(q)?;
    let v = Configuration::from_pyarray(v)?;
    let a = Configuration::from_pyarray(a)?;

    let tau = inverse_dynamics(&model.inner, &mut data.inner, &q, &v, &a).map_err(|e| {
        PyErr::new::<pyo3::exceptions::PyValueError, _>(format!("Error in inverse dynamics: {}", e))
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
