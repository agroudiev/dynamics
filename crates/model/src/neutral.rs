//! Algorithm to compute the neutral configuration of a model.

use crate::Configuration;
use crate::model::Model;
use crate::model::PyModel;
use numpy::{ToPyArray, ndarray::Array1};
use pyo3::prelude::*;

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
        q.rows_mut(offset, offset + joint_model.nq())
            .copy_from_slice(&q_joint);
        offset += joint_model.nq();
    }

    q
}

/// Python wrapper for the `neutral` function.
#[pyfunction(name = "neutral")]
pub fn py_neutral(py: Python, model: &mut PyModel) -> Py<PyAny> {
    let q = neutral(&mut model.inner);
    Array1::from_shape_vec(model.inner.nq, q.as_slice().to_vec())
        .unwrap()
        .to_pyarray(py)
        .into_any()
        .unbind()
}
