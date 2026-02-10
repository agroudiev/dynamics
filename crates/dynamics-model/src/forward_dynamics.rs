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

use crate::model::{Model, WORLD_ID};
use crate::{data::Data, errors::AlgorithmError};
use dynamics_joint::joint::JointModel;
use dynamics_joint::joint_data::JointData;
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
    let mut apparent_torque = tau.clone();
    // articulated body inertia matrix of the subtree in the local frame of the joint
    let mut aba_inertias = Vec::with_capacity(model.njoints() - 1); // skipping the world joint
    let mut joint_u = Vec::with_capacity(model.njoints() - 1); // skipping the world joint

    let mut q_offset = 0;
    let mut v_offset = 0;

    // forward pass 1
    #[allow(clippy::needless_range_loop)]
    for joint_id in 1..model.njoints() {
        // retrieve the joint model and the corresponding configuration
        let joint_model = &model.joint_models[joint_id];
        let joint_data = &mut data.joint_data[joint_id];
        let parent_id = model.joint_parents[joint_id];

        // extract the joint configuration, velocity and acceleration from configuration vectors
        let joint_q = q.rows(q_offset, joint_model.nq());
        let joint_v = v.rows(v_offset, joint_model.nv());

        joint_data
            .update(joint_model, &joint_q, Some(&joint_v))
            .unwrap();

        // update the local joint placement
        data.local_joint_placements[joint_id] =
            model.joint_placements[joint_id] * joint_data.get_joint_placement();

        // update the joint placement in the world frame
        data.joint_placements[joint_id] =
            data.joint_placements[parent_id] * data.local_joint_placements[joint_id];

        // update the Jacobian for the joint
        let column_data = joint_model.subspace_se3(&data.joint_placements[joint_id]);
        data.jacobian
            .update_column(v_offset, column_data.as_slice());

        // update the global joint velocity
        data.world_joint_velocities[joint_id] =
            data.joint_placements[joint_id].act(joint_data.get_joint_velocity());
        if parent_id != WORLD_ID {
            let (v_parent, v_child) = data.world_joint_velocities.split_at_mut(joint_id);
            v_child[0] += &v_parent[parent_id];
        }

        // update the global joint acceleration
        data.world_accelerations_gravity_field[joint_id] =
            data.joint_placements[joint_id].act(&joint_model.bias());
        if parent_id != WORLD_ID {
            data.world_accelerations_gravity_field[joint_id] +=
                data.world_joint_velocities[joint_id].cross(&data.world_joint_velocities[joint_id]);
        }

        // update inertias
        data.world_inertias[joint_id] =
            data.joint_placements[joint_id].act(&model.inertias[joint_id]);
        data.composite_inertias[joint_id] = data.world_inertias[joint_id].clone();
        aba_inertias.push(data.composite_inertias[joint_id].clone());

        // update forces
        data.world_joint_momenta[joint_id] =
            &data.world_inertias[joint_id] * &data.world_joint_velocities[joint_id];
        data.world_joint_forces[joint_id] =
            data.world_joint_velocities[joint_id].cross_force(&data.world_joint_momenta[joint_id]);

        q_offset += joint_model.nq();
        v_offset += joint_model.nv();
    }

    // backward pass
    for joint_id in (1..model.njoints()).rev() {
        let joint_model = &model.joint_models[joint_id];
        // let parent_id = model.joint_parents[joint_id];

        // check that nv = 1, otherwise u -= J^T * f is not well defined
        assert_eq!(
            joint_model.nv(),
            1,
            "The ABA implementation only supports joints with nv = 1 for now."
        );

        // update the apparent torque by subtracting the contribution of the spatial forces
        let mut u = apparent_torque.rows(v_offset, joint_model.nv());
        u -= &Configuration::from_row_slice(&[
            &data.jacobian.column(v_offset) * &data.world_joint_forces[joint_id]
        ]);
        apparent_torque
            .update_rows(v_offset, &u)
            .map_err(AlgorithmError::ConfigurationError)?;

        joint_u.push(&aba_inertias[joint_id - 1].matrix() * &data.jacobian.column(v_offset));

        v_offset -= joint_model.nv();
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
