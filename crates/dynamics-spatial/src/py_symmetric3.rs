use pyo3::{IntoPyObjectExt, prelude::*};

use crate::{py_vector3d::PyVector3D, symmetric3::Symmetric3};

/// Enum representing the possible input types for the `__mul__` method of `PySymmetric3`.
#[derive(FromPyObject)]
pub enum PySymmetric3Mul {
    Scalar(f64),
    Vector3D(PyVector3D),
}

/// A symmetric 3x3 matrix.
#[pyclass(name = "Symmetric3")]
#[derive(Debug, Clone, PartialEq)]
pub struct PySymmetric3 {
    inner: Symmetric3,
}

#[pymethods]
impl PySymmetric3 {
    #[new]
    #[must_use]
    /// Creates a new `Symmetric3` matrix from the given elements.
    pub fn from_elements(m11: f64, m22: f64, m33: f64, m12: f64, m13: f64, m23: f64) -> Self {
        PySymmetric3 {
            inner: Symmetric3::new(m11, m22, m33, m12, m13, m23),
        }
    }

    #[staticmethod]
    #[pyo3(name = "Zero")]
    /// Returns the zero symmetric matrix.
    pub fn zeros() -> Self {
        PySymmetric3 {
            inner: Symmetric3::zeros(),
        }
    }

    #[staticmethod]
    #[pyo3(name = "Identity")]
    /// Returns the identity symmetric matrix.
    pub fn identity() -> Self {
        PySymmetric3 {
            inner: Symmetric3::identity(),
        }
    }

    pub fn __add__(&self, other: &PySymmetric3) -> PySymmetric3 {
        PySymmetric3 {
            inner: self.inner + other.inner,
        }
    }

    pub fn __sub__(&self, other: &PySymmetric3) -> PySymmetric3 {
        PySymmetric3 {
            inner: self.inner - other.inner,
        }
    }

    pub fn __mul__(&self, py: Python, other: PySymmetric3Mul) -> Result<Py<PyAny>, PyErr> {
        match other {
            PySymmetric3Mul::Scalar(scalar) => PySymmetric3 {
                inner: self.inner * scalar,
            }
            .into_py_any(py),
            PySymmetric3Mul::Vector3D(vec) => PyVector3D {
                inner: &self.inner * &vec.inner,
            }
            .into_py_any(py),
        }
    }

    pub fn __rmul__(&self, scalar: f64) -> PySymmetric3 {
        PySymmetric3 {
            inner: self.inner * scalar,
        }
    }

    /// Converts the symmetric matrix to a 3x3 NumPy array.
    pub fn to_numpy(&self, py: Python) -> Py<PyAny> {
        self.inner.to_numpy(py)
    }

    /// Returns the symmetric matrix as a 3x3 NumPy array.
    pub fn matrix(&self, py: Python) -> Py<PyAny> {
        self.to_numpy(py)
    }

    fn __getitem__(&self, index: (usize, usize)) -> f64 {
        self.inner[index]
    }

    fn __repr__(&self) -> String {
        format!(
            "Symmetric3(\n  [{:.4}, {:.4}, {:.4}],\n  [{:.4}, {:.4}, {:.4}],\n  [{:.4}, {:.4}, {:.4}]\n)",
            self.inner[(0, 0)],
            self.inner[(0, 1)],
            self.inner[(0, 2)],
            self.inner[(1, 0)],
            self.inner[(1, 1)],
            self.inner[(1, 2)],
            self.inner[(2, 0)],
            self.inner[(2, 1)],
            self.inner[(2, 2)],
        )
    }
}
