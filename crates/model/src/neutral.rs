//! Algorithm to compute the neutral configuration of a model.

use crate::Configuration;
use crate::model::Model;
use crate::model::PyModel;
use numpy::{ToPyArray, ndarray::Array1};
use pyo3::prelude::*;

pub fn neutral(model: &mut Model) -> Configuration {
    let mut q = Configuration::zeros(model.nq);

    let mut offset = 0;
    for joint_model in model.iter_joint_models() {
        let q_joint = joint_model.neutral();
        q.rows_mut(offset, offset + joint_model.nq())
            .copy_from_slice(&q_joint);
        offset += joint_model.nq();
    }

    q
}

#[pyfunction(name = "neutral")]
pub fn py_neutral(py: Python, model: &mut PyModel) -> Py<PyAny> {
    let q = neutral(&mut model.inner);
    Array1::from_shape_vec(model.inner.nq, q.as_slice().to_vec())
        .unwrap()
        .to_pyarray(py)
        .into_any()
        .unbind()
}
