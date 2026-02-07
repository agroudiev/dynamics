//! Algorithm to compute the neutral configuration of a model.

use crate::model::Model;
use dynamics_joint::joint::JointModel;
use dynamics_spatial::configuration::Configuration;
use dynamics_spatial::configuration::ConfigurationError;

/// Computes the neutral configuration of a model.
///
/// This function iterates through each joint model in the given model, retrieves its neutral configuration, and assembles these into a single `Configuration` object that represents the neutral configuration of the entire model.
///
/// # Arguments
/// * `model` - A reference to the model.
///
/// # Returns
/// A `Configuration` object representing the neutral configuration of the model.
pub fn neutral(model: &Model) -> Result<Configuration, ConfigurationError> {
    let mut q = Configuration::zeros(model.nq);

    let mut offset = 0;
    for joint_model in &model.joint_models {
        let q_joint = joint_model.neutral();
        q.update_rows(offset, &q_joint)?;
        offset += joint_model.nq();
    }

    Ok(q)
}
