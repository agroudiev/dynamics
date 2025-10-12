//! Algorithm to compute the neutral configuration of a model.

use crate::model::Model;
use crate::model::PyModel;
use pyo3::prelude::*;
use spatial::configuration::Configuration;
use spatial::configuration::PyConfiguration;

/// Computes the neutral configuration of a model.
///
/// # Arguments
/// * `model` - A mutable reference to the model.
///
/// # Returns
/// A `Configuration` object representing the neutral configuration of the model.
pub fn neutral(model: &mut Model) -> Configuration {
    let mut q = Configuration::zeros(model.nq);

    let mut offset = 0;
    let mut keys = model.joint_models.keys().collect::<Vec<_>>();
    keys.sort();
    for joint_id in keys {
        let joint_model = model.joint_models.get(joint_id).unwrap();
        let q_joint = joint_model.neutral();
        q.update_rows(offset, &q_joint);
        offset += joint_model.nq();
    }

    q
}

/// Python wrapper for the `neutral` function.
#[pyfunction(name = "neutral")]
pub fn py_neutral(model: &mut PyModel) -> PyConfiguration {
    let q = neutral(&mut model.inner);
    PyConfiguration(q)
}
