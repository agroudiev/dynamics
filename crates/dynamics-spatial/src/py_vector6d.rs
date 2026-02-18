use pyo3::prelude::*;

use crate::vector6d::Vector6D;

/// A 6D vector representing spatial motion (angular and linear components).
///
/// A spatial vector is represented as a 6-dimensional vector,
/// which can be decomposed into [ω v], where ω is the angular component and v is the linear component.
#[pyclass(name = "Vector6D")]
pub struct PyVector6D {
    pub inner: Vector6D,
}

#[pymethods]
impl PyVector6D {
    #[new]
    /// Creates a new `Vector6D` from the given components.
    fn new(x: f64, y: f64, z: f64, w: f64, v: f64, u: f64) -> Self {
        Self {
            inner: Vector6D::new(x, y, z, w, v, u),
        }
    }

    pub fn __repr__(&self) -> String {
        format!("{}", self.inner)
    }
}
