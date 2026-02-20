//! Integration of joint configurations given their velocities.

use dynamics_joint::joint::JointModel;
use dynamics_spatial::configuration::Configuration;

use crate::{errors::AlgorithmError, model::Model};

/// Integrates the joint configurations given their velocities.
///
/// This function iterates through all the joints in the model, and integrates their configurations
/// using their respective `integrate` method. The resulting configuration is returned as a new `Configuration` object.
///
/// # Arguments
/// * `model` - The model containing the joint models to integrate.
/// * `q` - The current joint configuration of the model.
/// * `v` - The current joint velocity of the model.
///
/// # Returns
/// A new `Configuration` object containing the integrated joint configurations.
pub fn integrate(
    model: &Model,
    q: &Configuration,
    v: &Configuration,
) -> Result<Configuration, AlgorithmError> {
    let mut q_next = q.clone();
    let mut q_offset = 0;
    let mut v_offset = 0;

    for joint in model.joint_models.iter() {
        // recursively integrate the configuration of each joint
        q_next
            .update_rows(
                q_offset,
                &joint.integrate(&q.rows(q_offset, joint.nq()), &v.rows(v_offset, joint.nv())),
            )
            .map_err(AlgorithmError::ConfigurationError)?;

        q_offset += joint.nq();
        v_offset += joint.nv();
    }

    Ok(q_next)
}
