use pyo3::{exceptions::PyValueError, prelude::*};

use crate::vector3d::Vector3D;

#[pyclass(name = "Vector3D")]
#[derive(Debug, Clone, PartialEq)]
pub struct PyVector3D {
    pub inner: Vector3D,
}

#[pymethods]
impl PyVector3D {
    #[new]
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        PyVector3D {
            inner: Vector3D::new(x, y, z),
        }
    }

    #[staticmethod]
    pub fn zeros() -> Self {
        PyVector3D {
            inner: Vector3D::zeros(),
        }
    }

    #[staticmethod]
    pub fn ones() -> Self {
        PyVector3D {
            inner: Vector3D::new(1.0, 1.0, 1.0),
        }
    }

    pub fn __add__(&self, other: &PyVector3D) -> PyVector3D {
        PyVector3D {
            inner: self.inner + other.inner,
        }
    }

    pub fn __sub__(&self, other: &PyVector3D) -> PyVector3D {
        PyVector3D {
            inner: self.inner - other.inner,
        }
    }

    pub fn __mul__(&self, other: &PyVector3D) -> PyVector3D {
        PyVector3D {
            inner: self.inner * other.inner,
        }
    }

    pub fn to_numpy(&self, py: Python) -> Py<PyAny> {
        self.inner.to_numpy(py)
    }

    pub fn norm(&self) -> f64 {
        self.inner.norm()
    }

    pub fn vector(&self, py: Python) -> Py<PyAny> {
        self.inner.to_numpy(py)
    }

    pub fn __repr__(&self) -> String {
        format!(
            "Vector3D({}, {}, {})",
            self.inner.0.x, self.inner.0.y, self.inner.0.z
        )
    }

    pub fn __getitem__(&self, index: usize) -> PyResult<f64> {
        match index {
            0 => Ok(self.inner.0.x),
            1 => Ok(self.inner.0.y),
            2 => Ok(self.inner.0.z),
            _ => Err(PyValueError::new_err(format!(
                "Index out of bounds: {}. Valid indices are 0, 1, and 2.",
                index
            ))),
        }
    }
}
