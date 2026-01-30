use numpy::{PyReadonlyArrayDyn, ToPyArray, ndarray::Array1};
use pyo3::prelude::*;

use crate::{motion::SpatialMotion, py_vector3d::PyVector3D, vector3d::Vector3D};

#[pyclass(name = "SpatialMotion")]
pub struct PySpatialMotion {
    pub inner: SpatialMotion,
}

#[pymethods]
impl PySpatialMotion {
    #[new]
    pub fn new(angular: PyVector3D, linear: PyVector3D) -> Self {
        PySpatialMotion {
            inner: SpatialMotion::from_parts(angular.inner, linear.inner),
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
}
