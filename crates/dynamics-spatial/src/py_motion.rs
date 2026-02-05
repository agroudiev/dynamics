use numpy::{PyReadonlyArrayDyn, ToPyArray, ndarray::Array1};
use pyo3::prelude::*;

use crate::{
    motion::SpatialMotion, py_vector3d::PyVector3D, vector3d::Vector3D, vector6d::Vector6D,
};

#[pyclass(name = "SpatialMotion")]
pub struct PySpatialMotion {
    pub inner: SpatialMotion,
}

#[pymethods]
impl PySpatialMotion {
    #[new]
    pub fn new(array: PyReadonlyArrayDyn<f64>) -> PyResult<Self> {
        let array = array.as_array();
        if array.ndim() != 1 || array.len() != 6 {
            return Err(PyErr::new::<pyo3::exceptions::PyValueError, _>(
                "Input array must be one-dimensional with length 6.",
            ));
        }
        Ok(PySpatialMotion {
            inner: SpatialMotion(
                Vector6D::new(array[0], array[1], array[2], array[3], array[4], array[5]).0,
            ),
        })
    }

    #[staticmethod]
    pub fn from_vectors(linear: PyVector3D, angular: PyVector3D) -> Self {
        PySpatialMotion {
            inner: SpatialMotion::from_parts(linear.inner, angular.inner),
        }
    }

    #[staticmethod]
    /// Creates a new `SpatialMotion` object.
    /// # Arguments
    /// * `angular` - The angular component as a list of 3 floats.
    /// * `linear` - The linear component as a list of 3 floats.
    /// # Returns
    /// A new `SpatialMotion` object.
    pub fn from_parts(
        angular: PyReadonlyArrayDyn<f64>,
        linear: PyReadonlyArrayDyn<f64>,
    ) -> PyResult<Self> {
        Ok(PySpatialMotion {
            inner: SpatialMotion::from_parts(
                Vector3D::from_pyarray(&angular)?,
                Vector3D::from_pyarray(&linear)?,
            ),
        })
    }

    #[staticmethod]
    pub fn zero() -> Self {
        PySpatialMotion {
            inner: SpatialMotion::zero(),
        }
    }

    #[getter]
    pub fn translation(&self) -> PyVector3D {
        PyVector3D {
            inner: self.inner.translation(),
        }
    }

    #[getter]
    pub fn rotation(&self) -> PyVector3D {
        PyVector3D {
            inner: self.inner.rotation(),
        }
    }

    pub fn to_numpy(&self, py: Python) -> Py<PyAny> {
        Array1::from_iter(self.inner.0.iter().copied())
            .to_pyarray(py)
            .into_any()
            .unbind()
    }

    pub fn cross(&self, other: &PySpatialMotion) -> PySpatialMotion {
        PySpatialMotion {
            inner: SpatialMotion(self.inner.cross(&other.inner).0),
        }
    }

    pub fn cross_star(&self, other: &PySpatialMotion) -> PySpatialMotion {
        PySpatialMotion {
            inner: SpatialMotion(self.inner.cross_star(&other.inner).0),
        }
    }

    pub fn __repr__(&self) -> PyResult<String> {
        Ok(format!("{}", self.inner))
    }

    pub fn __add__(&self, other: &PySpatialMotion) -> PySpatialMotion {
        PySpatialMotion {
            inner: SpatialMotion(self.inner.0 + other.inner.0),
        }
    }
}
