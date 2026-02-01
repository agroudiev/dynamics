//! Forward kinematics algorithms for robot models.
//!
//! ## Overview
//! The forward kinematics algorithm computes the position, orientation,
//! velocity, and acceleration of each joint and frame in the robot model.
//!
//! In particular, the following quantities are computed and stored in the `Data` structure:
//! - `data.joint_placements`: The placement of each joint in the world frame.
//! - `data.local_joint_placements`: The placement of each joint in its parent joint frame.
//! - `data.joint_velocities`: The velocity of each joint in the world frame.
//! - `data.joint_accelerations`: The acceleration of each joint in the world frame.
//!
//! Each joint's data (accessible via `data.joint_data`) is also updated.
//!
//! ## Frames
//! By default, only joint placements are computed. To compute frame placements,
//! the `update_frame_placements` function must be called after `forward_kinematics`.
//! This function updates the `data.frame_placements` based on the current joint placements.

use crate::data::Data;
use crate::errors::AlgorithmError;
use crate::model::Model;
use dynamics_joint::joint::JointModel;
use dynamics_joint::joint_data::JointData;
use dynamics_spatial::configuration::Configuration;
use dynamics_spatial::se3::SE3;

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
/// * Returns an [`AlgorithmError`] if there was an error.
pub fn forward_kinematics(
    model: &Model,
    data: &mut Data,
    q: &Configuration,
    v: &Option<Configuration>,
    a: &Option<Configuration>,
) -> Result<(), AlgorithmError> {
    // check if configurations are of the right size
    q.check_size("q", model.nq)
        .map_err(AlgorithmError::ConfigurationError)?;
    v.as_ref()
        .map(|v| {
            v.check_size("v", model.nv)
                .map_err(AlgorithmError::ConfigurationError)
        })
        .transpose()?;
    a.as_ref()
        .map(|a| {
            a.check_size("a", model.nv)
                .map_err(AlgorithmError::ConfigurationError)
        })
        .transpose()?;

    // update the joints data
    let mut q_offset = 0;
    let mut v_offset = 0;
    for id in 0..model.njoints() {
        // retrieve joint data and model
        let joint_data = &mut data.joint_data[id];
        let joint_model = &model.joint_models[id];

        // extract the joint configuration and velocity
        let q_joint = q.rows(q_offset, joint_model.nq());
        let v_joint = v.as_ref().map(|v| v.rows(v_offset, joint_model.nv()));
        let a_joint = a.as_ref().map(|a| a.rows(v_offset, joint_model.nv()));

        // update the joint data
        match joint_data.update(joint_model, &q_joint, v_joint.as_ref()) {
            Ok(()) => {}
            Err(e) => {
                return Err(AlgorithmError::JointError(model.joint_names[id].clone(), e));
            }
        }

        // update the data velocity
        if v.is_some() {
            data.joint_velocities[id] = joint_data.get_joint_velocity().clone();
        }

        // update the data acceleration
        if let Some(a_joint) = a_joint {
            data.joint_accelerations[id] = joint_model.subspace(&a_joint) + joint_model.bias();
        }

        q_offset += joint_model.nq();
        v_offset += joint_model.nv();
    }

    // update the placements of the joints in the world frame
    // by traversing the joint tree
    data.joint_placements = vec![SE3::identity()];

    for joint_id in 1..model.njoints() {
        // get the placement of the parent joint in the world frame
        let parent_id = model.joint_parents[joint_id]; // we checked that the parent existed before
        let parent_placement = data.joint_placements[parent_id];

        // get the placement of the joint in the parent frame
        let local_joint_placement = model.joint_placements[joint_id];

        // get the joint transformation
        let joint_data = &data.joint_data[joint_id];
        let joint_placement = joint_data.get_joint_placement();

        // compute the local joint placement in the parent frame
        data.local_joint_placements[joint_id] = local_joint_placement * joint_placement;

        // compute the placement of the joint in the world frame
        data.joint_placements.insert(
            joint_id,
            parent_placement * data.local_joint_placements[joint_id],
        );

        // update the joint velocity if v is provided
        if v.is_some() {
            // split borrow to update joint velocities
            let (v_parent, v_child) = data.joint_velocities.split_at_mut(joint_id);
            // update the joint velocity of the joint at joint_id
            v_child[0] += data.local_joint_placements[joint_id].act_inv(&v_parent[parent_id]);
        }

        // update the joint acceleration if a is provided
        if a.is_some() {
            // split borrow to update joint velocities
            let (a_parent, a_child) = data.joint_accelerations.split_at_mut(joint_id);
            // update the joint velocity of the joint at joint_id
            a_child[0] += data.local_joint_placements[joint_id].act_inv(&a_parent[parent_id])
                + (data.joint_velocities[joint_id].cross(joint_data.get_joint_velocity()));
        }
    }

    Ok(())
}

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
pub fn update_frame_placements(model: &Model, data: &mut Data) {
    for frame_id in 1..model.nframes() {
        let frame = &model.frames[frame_id];
        let parent_joint_placement = data.joint_placements[frame.parent_joint];
        data.frame_placements[frame_id] = parent_joint_placement * frame.placement;
    }
}
