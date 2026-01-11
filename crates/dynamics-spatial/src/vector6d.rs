//! Defines **spatial (6D) vectors** and related operations.

use nalgebra::{Matrix6, Vector6};
use pyo3::prelude::*;

#[derive(Debug, Clone, Copy, PartialEq)]
/// A 6D vector representing spatial motion (angular and linear components).
///
/// A spatial vector is represented as a 6-dimensional vector,
/// which can be decomposed into $\begin{bmatrix} \omega & v \end{bmatrix}$,
/// where $\omega$ is the angular component and $v$ is the linear component.
pub struct Vector6D(pub(crate) Vector6<f64>);

impl Vector6D {
    pub fn new(x: f64, y: f64, z: f64, w: f64, v: f64, u: f64) -> Self {
        Self(Vector6::new(x, y, z, w, v, u))
    }

    pub fn zeros() -> Self {
        Self(Vector6::zeros())
    }

    pub fn as_slice(&self) -> &[f64; 6] {
        self.0.as_slice().try_into().unwrap()
    }

    /// Returns the vector as a diagonal 6x6 matrix.
    pub fn as_diagonal(&self) -> Matrix6<f64> {
        Matrix6::from_diagonal(&self.0)
    }
}

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
