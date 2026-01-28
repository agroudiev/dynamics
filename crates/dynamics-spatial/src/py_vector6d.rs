use pyo3::prelude::*;

use crate::vector6d::Vector6D;

/// Python wrapper for spatial motion vectors (`Vector6D`).
#[pyclass(name = "Vector6D")]
pub struct PyVector6D {
    pub inner: Vector6D,
}

#[pymethods]
impl PyVector6D {
    #[new]
    fn new(x: f64, y: f64, z: f64, w: f64, v: f64, u: f64) -> Self {
        Self {
            inner: Vector6D::new(x, y, z, w, v, u),
        }
    }
}
