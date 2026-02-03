//! This module contains the implementation of the inverse dynamics algorithms.
//! The main algorithm implemented here is the Recursive Newton-Euler Algorithm (RNEA).
//! The RNEA computes the joint torques required to achieve a given motion of the robot
//! given its configuration, velocity, and acceleration.

use crate::data::Data;
use crate::model::{Model, WORLD_ID};
use dynamics_joint::joint::JointModel;
use dynamics_joint::joint_data::JointData;
use dynamics_spatial::configuration::{Configuration, ConfigurationError};
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
/// * `Ok(())` if the inverse dynamics was successful.
/// * `Err(ConfigurationError)` if there was an error.
pub fn inverse_dynamics(
    model: &Model,
    data: &mut Data,
    q: &Configuration,
    v: &Configuration,
    a: &Configuration,
) -> Result<(), ConfigurationError> {
    let mut offset = 0;

    data.joint_velocities[0] = SpatialMotion::zero();
    data.joint_accelerations[0] = SpatialMotion::from_parts(Vector3D::zeros(), model.gravity);

    // Forward pass: compute velocities and accelerations
    for joint_id in 0..model.joint_models.len() {
        // retrieve the joint model and the corresponding configuration
        let joint_model = &model.joint_models[joint_id];
        let joint_data = &mut data.joint_data[joint_id];
        let parent_id = model.joint_parents[joint_id];
        // extract the joint configuration, velocity and acceleration from configuration vectors
        let q_joint = q.rows(offset, joint_model.nq());
        let v_joint = v.rows(offset, joint_model.nq());
        let a_joint = a.rows(offset, joint_model.nq());

        // compute the transformation matrix of the joint (X_J) and axis (S_i)
        // let transform = joint_model.transform(&q_joint);
        // let axis = joint_model.get_axis();

        joint_data
            .update(joint_model, &q_joint, Some(&v_joint))
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
        let (a_parent, a_child) = data.joint_accelerations.split_at_mut(joint_id);

        a_child[0] = joint_model.subspace(&a_joint) + joint_model.bias();
        a_child[0] += data.local_joint_placements[joint_id].act_inv(&a_parent[parent_id]);
        a_child[0] += data.joint_velocities[joint_id].cross(joint_data.get_joint_velocity());

        // update the joint momentum
        data.joint_momenta[joint_id] = &model.inertias[joint_id] * &data.joint_velocities[joint_id];

        // update the joint force
        data.joint_forces[joint_id] =
            &model.inertias[joint_id] * &data.joint_accelerations[joint_id];
        data.joint_forces[joint_id] +=
            data.joint_velocities[joint_id].cross(&data.joint_momenta[joint_id]);

        offset += joint_model.nq();
    }

    // TODO: add external forces

    // Backward pass: compute the joint torques
    for joint_id in (0..model.joint_models.len()).rev() {
        let joint_model = &model.joint_models[joint_id];
        let parent_id = model.joint_parents[joint_id];
        offset -= joint_model.nq();

        data.tau.update_rows(
            offset,
            &joint_model.subspace_dual(&data.joint_forces[joint_id]),
        )?;

        if parent_id != WORLD_ID {
            let (f_parent, f_child) = data.joint_forces.split_at_mut(joint_id);
            f_parent[parent_id] += data.local_joint_placements[joint_id].act(&f_child[0]);
        }
    }

    Ok(())
}
