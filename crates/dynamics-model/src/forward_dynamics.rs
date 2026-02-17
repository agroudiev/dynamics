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
use dynamics_inertia::inertia::InertiaMatrix;
use dynamics_joint::joint::JointModel;
use dynamics_joint::joint_data::JointData;
use dynamics_spatial::configuration::Configuration;
use dynamics_spatial::force::SpatialForce;
use dynamics_spatial::motion::SpatialMotion;
use dynamics_spatial::vector3d::Vector3D;
use dynamics_spatial::vector6d::Vector6D;

/// Convention for the ABA algorithm.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg(feature = "python")]
#[pyo3::pyclass(name = "ABAConvention")]
pub enum ABAConvention {
    Local,
    World,
}

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
pub fn forward_dynamics<'a>(
    model: &Model,
    data: &'a mut Data,
    q: &Configuration,
    v: &Configuration,
    tau: &Configuration,
    convention: ABAConvention,
) -> Result<&'a Configuration, AlgorithmError> {
    match convention {
        ABAConvention::Local => forward_dynamics_local(model, data, q, v, tau),
        ABAConvention::World => forward_dynamics_world(model, data, q, v, tau),
    }
}

/// Computes the forward dynamics of the robot model using the Articulated Body Algorithm (ABA) in local convention.
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
pub fn forward_dynamics_local<'a>(
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

    // initialize the acceleration, force, and velocity of the world joint
    data.joint_accelerations_gravity_field[0] =
        SpatialMotion::from_parts(-model.gravity, Vector3D::zeros());
    data.joint_forces[0] = SpatialForce::zero();
    data.joint_velocities[0] = SpatialMotion::zero();

    // initialize the apparent torque (a.k.a. u) with the input torque
    let mut apparent_torque = tau.clone();
    // articulated body inertia matrix of the subtree in the local frame of the joint
    let mut aba_inertias = vec![InertiaMatrix::zeros(); model.njoints()];
    let mut joint_u = vec![Vector6D::zeros(); model.njoints()];
    // let mut joint_stu = vec![0.0; model.njoints()];
    let mut joint_dinv = vec![0.0; model.njoints()];
    let mut joint_udinv = vec![Vector6D::zeros(); model.njoints()];
    // TODO: make these vectors of size nv instead of njoints and handle the indexing properly

    let mut q_offset = 0;
    let mut v_offset = 0;

    // forward pass 1
    #[allow(clippy::needless_range_loop)]
    for joint_id in 1..model.njoints() {
        // retrieve the joint model and the corresponding configuration
        let joint_model = &model.joint_models[joint_id];
        let joint_data = &mut data.joint_data[joint_id];
        let parent_id = model.joint_parents[joint_id];

        // check that nv <= 1, otherwise the algorithm needs to be modified
        assert!(
            joint_model.nv() <= 1,
            "The ABA implementation only supports joints with nv = 1 for now."
        );

        // extract the joint configuration, velocity and acceleration from configuration vectors
        let joint_q = q.rows(q_offset, joint_model.nq());
        let joint_v = v.rows(v_offset, joint_model.nv());

        joint_data
            .update(joint_model, &joint_q, Some(&joint_v))
            .unwrap();

        // update the local joint placement
        data.local_joint_placements[joint_id] =
            model.joint_placements[joint_id] * joint_data.get_joint_placement();

        // update the joint velocity
        let (v_parent, v_child) = data.joint_velocities.split_at_mut(joint_id);
        v_child[0] = joint_data.get_joint_velocity().clone();
        if parent_id != WORLD_ID {
            v_child[0] += data.local_joint_placements[joint_id].act_inv(&v_parent[parent_id]);
        }

        // update the joint acceleration
        data.joint_accelerations_gravity_field[joint_id] = joint_model.bias()
            + data.joint_velocities[joint_id].cross(joint_data.get_joint_velocity());

        // update inertias
        aba_inertias[joint_id] = model.inertias[joint_id].matrix();

        // update momenta and forces
        data.joint_momenta[joint_id] = &model.inertias[joint_id] * &data.joint_velocities[joint_id];
        data.joint_forces[joint_id] =
            data.joint_velocities[joint_id].cross_force(&data.joint_momenta[joint_id]);

        // TODO: handle external forces

        q_offset += joint_model.nq();
        v_offset += joint_model.nv();
    }

    // backward pass
    for joint_id in (1..model.njoints()).rev() {
        let joint_model = &model.joint_models[joint_id];
        let parent_id = model.joint_parents[joint_id];
        v_offset -= joint_model.nv();

        // update the apparent torque by subtracting the contribution of the spatial forces
        if joint_model.nv() > 0 {
            let mut u = apparent_torque.rows(v_offset, joint_model.nv());
            u -= &joint_model.subspace_dual(&data.joint_forces[joint_id]);
            apparent_torque
                .update_rows(v_offset, &u)
                .map_err(AlgorithmError::ConfigurationError)?;

            // set intermediate quantities
            let axis = &joint_model.get_axis()[0];
            let u = &aba_inertias[joint_id] * axis;
            joint_u[joint_id] = Vector6D::from_slice(u.0.as_slice().try_into().unwrap());
            joint_dinv[joint_id] = 1.0 / axis.0.dot(&joint_u[joint_id].0);
            joint_udinv[joint_id] = &joint_u[joint_id] * joint_dinv[joint_id];
            // TODO: this is ugly, cleanup code

            // update aba_inertias
            if parent_id != WORLD_ID {
                aba_inertias[joint_id] -=
                    InertiaMatrix::from_vectors(&joint_udinv[joint_id], &joint_u[joint_id]);
            }

            // TODO: handle armature term
        }

        if parent_id != WORLD_ID && joint_model.nv() > 0 {
            // update forces
            data.joint_forces[joint_id] +=
                &aba_inertias[joint_id] * &data.joint_accelerations_gravity_field[joint_id];
            let u = apparent_torque.rows(v_offset, joint_model.nv())[0];
            data.joint_forces[joint_id] += SpatialForce::from_vector6d(u * &joint_udinv[joint_id]);

            // update aba_inertias
            let (aba_parent, aba_child) = aba_inertias.split_at_mut(joint_id);
            aba_parent[parent_id] +=
                aba_child[0].transform_frame(&data.local_joint_placements[joint_id]);

            // update parent forces
            let (f_parent, f_child) = data.joint_forces.split_at_mut(joint_id);
            f_parent[parent_id] += data.local_joint_placements[joint_id].act(&f_child[0]);
        }
    }

    // forward pass 2
    for joint_id in 1..model.njoints() {
        let joint_model = &model.joint_models[joint_id];
        let parent_id = model.joint_parents[joint_id];

        // update acceleration
        let (a_parent, a_child) = data
            .joint_accelerations_gravity_field
            .split_at_mut(joint_id);
        a_child[0] += data.local_joint_placements[joint_id].act_inv(&a_parent[parent_id]);

        if joint_model.nv() > 0 {
            // update ddq
            let joint_ddq = joint_dinv[joint_id]
                * apparent_torque.rows(v_offset, joint_model.nv())[0]
                - joint_udinv[joint_id].0.dot(&a_child[0].0);
            // TODO: do not use .0 here
            let joint_ddq = Configuration::from_row_slice(&[joint_ddq]);
            data.ddq
                .update_rows(v_offset, &joint_ddq)
                .map_err(AlgorithmError::ConfigurationError)?;

            // add ddq term to the acceleration
            a_child[0] += joint_model.subspace(&joint_ddq);
        }

        // put the currently computed acceleration to the non-gravity field
        data.joint_accelerations[joint_id] =
            data.joint_accelerations_gravity_field[joint_id].clone();

        // add gravity compensation term
        let linear = data.joint_placements[joint_id].rotation().transpose() * &model.gravity;
        data.joint_accelerations[joint_id] += SpatialMotion::from_parts(linear, Vector3D::zeros());

        // update the force
        data.joint_forces[joint_id] = &model.inertias[joint_id]
            * &data.joint_accelerations_gravity_field[joint_id]
            + data.joint_velocities[joint_id].cross_force(&data.joint_momenta[joint_id]);

        v_offset += joint_model.nv();
    }

    // update the forces
    for (joint_id, parent_id) in model.joint_parents.iter().enumerate().skip(1).rev() {
        let (f_parent, f_child) = data.joint_forces.split_at_mut(joint_id);
        f_parent[*parent_id] += data.local_joint_placements[joint_id].act(&f_child[0]);
    }

    Ok(&data.ddq)
}

/// Computes the forward dynamics of the robot model using the Articulated Body Algorithm (ABA) in world convention.
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
pub fn forward_dynamics_world<'a>(
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
    data.joint_accelerations_gravity_field[0] =
        SpatialMotion::from_parts(-model.gravity, Vector3D::zeros());
    data.world_joint_forces[0] = SpatialForce::zero();

    // initialize the apparent torque (a.k.a. u) with the input torque
    let mut apparent_torque = tau.clone();
    // articulated body inertia matrix of the subtree in the local frame of the joint
    let mut aba_inertias = vec![InertiaMatrix::zeros(); model.njoints()];
    let mut joint_u = vec![Vector6D::zeros(); model.njoints()];
    let mut joint_stu = vec![0.0; model.njoints()];
    let mut joint_dinv = vec![0.0; model.njoints()];
    let mut joint_udinv = vec![Vector6D::zeros(); model.njoints()];
    // TODO: make these vectors of size nv instead of njoints and handle the indexing properly

    let mut q_offset = 0;
    let mut v_offset = 0;

    // forward pass 1
    #[allow(clippy::needless_range_loop)]
    for joint_id in 1..model.njoints() {
        // retrieve the joint model and the corresponding configuration
        let joint_model = &model.joint_models[joint_id];
        let joint_data = &mut data.joint_data[joint_id];
        let parent_id = model.joint_parents[joint_id];

        // check that nv <= 1, otherwise the algorithm needs to be modified
        assert!(
            joint_model.nv() <= 1,
            "The ABA implementation only supports joints with nv = 1 for now."
        );

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
        if joint_model.nv() > 0 {
            let column_data = joint_model.subspace_se3(&data.joint_placements[joint_id]);
            data.jacobian
                .update_column(v_offset, column_data.as_slice());
        }

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
        aba_inertias[joint_id] = data.composite_inertias[joint_id].matrix();

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
        let parent_id = model.joint_parents[joint_id];
        v_offset -= joint_model.nv();

        // update the apparent torque by subtracting the contribution of the spatial forces
        let mut u = apparent_torque.rows(v_offset, joint_model.nv());
        u -= &Configuration::from_row_slice(&[
            &data.jacobian.column(v_offset) * &data.world_joint_forces[joint_id]
        ]);
        apparent_torque
            .update_rows(v_offset, &u)
            .map_err(AlgorithmError::ConfigurationError)?;
        // FIXME: I wonder if the code above will work, but it might

        // update intermediate quantities
        if joint_model.nv() > 0 {
            joint_u[joint_id] = &aba_inertias[joint_id] * &data.jacobian.column(v_offset);
            joint_stu[joint_id] = &data.jacobian.column(v_offset) * &joint_u[joint_id];

            // TODO: add armature term

            joint_dinv[joint_id] = 1.0 / (joint_stu[joint_id]);
            joint_udinv[joint_id] = &joint_u[joint_id] * joint_dinv[joint_id];
        }

        if parent_id != WORLD_ID {
            if joint_model.nv() > 0 {
                aba_inertias[joint_id] -=
                    InertiaMatrix::from_vectors(&joint_udinv[joint_id], &joint_u[joint_id]);

                // update the joint force
                data.world_joint_forces[joint_id] +=
                    &aba_inertias[joint_id] * &data.world_accelerations_gravity_field[joint_id];
                let u = apparent_torque.rows(v_offset, joint_model.nv())[0];
                data.world_joint_forces[joint_id] +=
                    SpatialForce::from_vector6d(&joint_udinv[joint_id] * u);

                // update aba_inertias
                let (i_parent, i_child) = aba_inertias.split_at_mut(joint_id);
                i_parent[parent_id] += i_child[0].clone();
            }

            // update the parent joint force
            let (f_parent, f_child) = data.world_joint_forces.split_at_mut(joint_id);
            f_parent[parent_id] += &f_child[0];
        }
    }

    // forward pass 2
    for joint_id in 1..model.njoints() {
        let joint_model = &model.joint_models[joint_id];
        let parent_id = model.joint_parents[joint_id];

        // update acceleration
        let (a_parent, a_child) = data
            .world_accelerations_gravity_field
            .split_at_mut(joint_id);
        a_child[0] += data.local_joint_placements[joint_id].act_inv(&a_parent[parent_id]);

        if joint_model.nv() > 0 {
            // update ddq
            let joint_ddq = joint_dinv[joint_id]
                * apparent_torque.rows(v_offset, joint_model.nv())[0]
                - joint_udinv[joint_id].0.dot(&a_child[0].0);
            // TODO: do not use .0 here
            let joint_ddq = Configuration::from_row_slice(&[joint_ddq]);
            data.ddq
                .update_rows(v_offset, &joint_ddq)
                .map_err(AlgorithmError::ConfigurationError)?;

            // add ddq term to the acceleration
            a_child[0] += joint_model.subspace(&joint_ddq);
        }

        // add gravity compensation term
        let linear = data.joint_placements[joint_id].rotation().transpose() * &model.gravity;
        a_child[0] += SpatialMotion::from_parts(linear, Vector3D::zeros());

        // update the force
        data.joint_forces[joint_id] = &model.inertias[joint_id]
            * &data.joint_accelerations_gravity_field[joint_id]
            + data.joint_velocities[joint_id].cross_force(&data.joint_momenta[joint_id]);

        v_offset += joint_model.nv();
    }

    // update the forces
    for (joint_id, parent_id) in model.joint_parents.iter().enumerate().skip(1).rev() {
        let (parent, child) = data.world_joint_forces.split_at_mut(joint_id);
        parent[*parent_id] += &child[0];
    }

    Ok(&data.ddq)
}

#[cfg(test)]
mod tests {
    use super::*;
    use dynamics_joint::{joint::JointWrapper, revolute::JointModelRevolute};
    use dynamics_spatial::se3::SE3;

    #[test]
    fn test_fd_one_joint() {
        let mut model = Model::new_empty();
        let joint_model = JointWrapper::revolute(JointModelRevolute::new_rx());
        model
            .add_joint(0, joint_model, SE3::identity(), "joint1".to_string())
            .unwrap();

        let mut data = model.create_data();

        let q = Configuration::from_row_slice(&[0.5]);
        let v = Configuration::from_row_slice(&[0.2]);
        let tau = Configuration::from_row_slice(&[1.0]);

        let _ddq = forward_dynamics(&model, &mut data, &q, &v, &tau, ABAConvention::Local).unwrap();
    }
}
