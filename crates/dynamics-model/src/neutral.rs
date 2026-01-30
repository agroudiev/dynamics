//! Algorithm to compute the neutral configuration of a model.

use crate::model::Model;
use dynamics_joint::joint::JointModel;
use dynamics_spatial::configuration::Configuration;
use dynamics_spatial::configuration::ConfigurationError;

/// Computes the neutral configuration of a model.
///
/// # Arguments
/// * `model` - A mutable reference to the model.
///
/// # Returns
/// A `Configuration` object representing the neutral configuration of the model.
pub fn neutral(model: &mut Model) -> Result<Configuration, ConfigurationError> {
    let mut q = Configuration::zeros(model.nq);

    let mut offset = 0;
    for joint_model in &model.joint_models {
        let q_joint = joint_model.neutral();
        q.update_rows(offset, &q_joint)?;
        offset += joint_model.nq();
    }

    Ok(q)
}
