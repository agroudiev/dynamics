use crate::Configuration;
use crate::data::{Data, PyData};
use crate::model::{Model, PyModel};
use numpy::PyReadonlyArray1;
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

pub fn foward_dynamics(model: &mut Model, data: &Data, q: &Configuration) {
    // TODO: check if q is of the right size
}

#[pyfunction(name = "forward_dynamics")]
pub fn py_forward_dynamics(
    model: &mut PyModel,
    data: &PyData,
    q: PyReadonlyArray1<f64>,
) -> PyResult<()> {
    let q = q.as_array();
    if q.shape() != [model.inner.nq] {
        return Err(PyValueError::new_err(format!(
            "Invalid input size. Expected a configuration of size {}, got {:?}",
            model.inner.nq,
            q.shape()
        )));
    }
    let q = match q.as_slice() {
        Some(slice) => slice,
        None => return Err(PyValueError::new_err("Failed to convert q to slice")),
    };
    let q = Configuration::from_row_slice(q);

    foward_dynamics(&mut model.inner, &data.inner, &q);
    Ok(())
}
