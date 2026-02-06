//! This module contains the implementation of the inverse dynamics algorithms.
//!
//! Inverse dynamics is the problem of computing the joint torques required to achieve a given motion of the robot: $$\tau = \text{ID}(q, \dot{q}, \ddot{q})$$
//! where $q$ is the configuration of the robot, $\dot{q}$ is the velocity of the robot, and $\ddot{q}$ is the acceleration of the robot.
//!
//! The main algorithm implemented here is the Recursive Newton-Euler Algorithm (RNEA), having a complexity of $O(n)$ in the number of joints.
//! It uses two passes over the robot model:
//! a forward pass to compute the velocities and accelerations of each joint,
//! and a backward pass to compute the joint torques.
//! Both passes are implemented in the `inverse_dynamics` function.

use crate::data::Data;
use crate::errors::AlgorithmError;
use crate::model::{Model, WORLD_ID};
use dynamics_joint::joint::JointModel;
use dynamics_joint::joint_data::JointData;
use dynamics_spatial::configuration::Configuration;
use dynamics_spatial::motion::SpatialMotion;
use dynamics_spatial::vector3d::Vector3D;

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
/// * `Ok(())` if the inverse dynamics was successful. In this case, the fields `tau`, `local_joint_placements`, `joint_velocities`, `joint_accelerations_gravity_free`, `joint_momenta` and `joint_forces` of the `data` structure are updated with the results of the algorithm.
/// * `Err(ConfigurationError)` if there was an error.
pub fn inverse_dynamics(
    model: &Model,
    data: &mut Data,
    q: &Configuration,
    v: &Configuration,
    a: &Configuration,
) -> Result<(), AlgorithmError> {
    let mut q_offset = 0;
    let mut v_offset = 0;

    data.joint_velocities[0] = SpatialMotion::zero();
    data.joint_accelerations_gravity_free[0] =
        SpatialMotion::from_parts(-model.gravity, Vector3D::zeros());

    // Forward pass: compute velocities and accelerations
    for joint_id in 1..model.joint_models.len() {
        // retrieve the joint model and the corresponding configuration
        let joint_model = &model.joint_models[joint_id];
        let joint_data = &mut data.joint_data[joint_id];
        let parent_id = model.joint_parents[joint_id];

        // extract the joint configuration, velocity and acceleration from configuration vectors
        let joint_q = q.rows(q_offset, joint_model.nq());
        let joint_v = v.rows(v_offset, joint_model.nv());
        let joint_a = a.rows(v_offset, joint_model.nv());

        joint_data
            .update(joint_model, &joint_q, Some(&joint_v))
            .unwrap();

        // update the joint placement in the world frame
        data.local_joint_placements[joint_id] =
            model.joint_placements[joint_id] * joint_data.get_joint_placement();

        // update the joint velocity
        data.joint_velocities[joint_id] = joint_data.get_joint_velocity().clone();
        if parent_id != WORLD_ID {
            let (v_parent, v_child) = data.joint_velocities.split_at_mut(joint_id);
            v_child[0] += data.local_joint_placements[joint_id].act_inv(&v_parent[parent_id]);
        }

        // update the joint acceleration
        let (a_parent, a_child) = data.joint_accelerations_gravity_free.split_at_mut(joint_id);

        a_child[0] = joint_model.subspace(&joint_a) + joint_model.bias();
        a_child[0] += data.local_joint_placements[joint_id].act_inv(&a_parent[parent_id]);
        a_child[0] += data.joint_velocities[joint_id].cross(joint_data.get_joint_velocity());

        // update the joint momentum
        data.joint_momenta[joint_id] = &model.inertias[joint_id] * &data.joint_velocities[joint_id];

        // update the joint force
        data.joint_forces[joint_id] =
            &model.inertias[joint_id] * &data.joint_accelerations_gravity_free[joint_id];
        data.joint_forces[joint_id] +=
            data.joint_velocities[joint_id].cross_force(&data.joint_momenta[joint_id]);

        q_offset += joint_model.nq();
        v_offset += joint_model.nv();
    }

    // TODO: add external forces

    // Backward pass: compute the joint torques
    for joint_id in (1..model.joint_models.len()).rev() {
        let joint_model = &model.joint_models[joint_id];
        let parent_id = model.joint_parents[joint_id];
        v_offset -= joint_model.nv();

        data.tau
            .update_rows(
                v_offset,
                &joint_model.subspace_dual(&data.joint_forces[joint_id]),
            )
            .map_err(AlgorithmError::ConfigurationError)?;

        if parent_id != WORLD_ID {
            let (f_parent, f_child) = data.joint_forces.split_at_mut(joint_id);
            f_parent[parent_id] += data.local_joint_placements[joint_id].act(&f_child[0]);
        }
    }

    Ok(())
}
