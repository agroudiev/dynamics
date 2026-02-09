//! This module contains the implementation of the forward dynamics algorithms.
//!
//! Forward dynamics is the problem of computing the joint accelerations of the robot given the configuration, velocity, and joint torques: $$\ddot{q} = \text{FD}(q, \dot{q}, \tau)$$
//! It is mainly used for simulation purposes: this is repeatedly invoked
//! at each time step to compute the accelerations, which are then integrated to compute the velocities and configurations at the next time step.
//!
//! The main algorithm implemented here is the Articulated Body Algorithm (ABA), having a complexity of $O(n)$ in the number of joints.
//! It uses three passes over the robot model:
//! - a first forward pass to compute the velocities of each joint as well as the forces,
//! - a backward pass to compute the articulated body inertias and the bias forces,
//! - a second forward pass to compute the joint accelerations.
//!
//! All three passes are implemented in the `forward_dynamics` function.

use crate::model::Model;
use crate::{data::Data, errors::AlgorithmError};
use dynamics_spatial::configuration::Configuration;
use dynamics_spatial::force::SpatialForce;
use dynamics_spatial::motion::SpatialMotion;
use dynamics_spatial::vector3d::Vector3D;

/// Computes the forward dynamics of the robot model using the Articulated Body Algorithm (ABA).
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
/// * `Ok(ddq)` if the forward dynamics was successful. The following fields of the `data` structure will be updated:
///     - **TODO**
/// * `Err(ConfigurationError)` if there was an error.
pub fn forward_dynamics<'a>(
    model: &Model,
    data: &'a mut Data,
    q: &Configuration,
    v: &Configuration,
    tau: &Configuration,
) -> Result<&'a Configuration, AlgorithmError> {
    // check the dimensions of the input
    q.check_size("q", model.nq)
        .map_err(AlgorithmError::ConfigurationError)?;
    v.check_size("v", model.nv)
        .map_err(AlgorithmError::ConfigurationError)?;
    tau.check_size("tau", model.nv)
        .map_err(AlgorithmError::ConfigurationError)?;

    // initialize the world acceleration and force
    data.world_accelerations_gravity_field[0] =
        SpatialMotion::from_parts(-model.gravity, Vector3D::zeros());
    data.world_joint_forces[0] = SpatialForce::zero();
    // initialize the apparent torque (a.k.a. u) with the input torque
    // let mut apparent_torque = tau.clone();

    // forward pass 1
    for _joint_id in 1..model.njoints() {
        // TODO
    }

    // backward pass
    for _joint_id in (1..model.njoints()).rev() {
        // TODO
    }

    // forward pass 2
    for _joint_id in 1..model.njoints() {
        // TODO
    }

    // update the forces
    for (joint_id, parent_id) in model.joint_parents.iter().enumerate().skip(1).rev() {
        let (parent, child) = data.world_joint_forces.split_at_mut(joint_id);
        parent[*parent_id] += &child[0];
    }

    Ok(&data.ddq)
}
