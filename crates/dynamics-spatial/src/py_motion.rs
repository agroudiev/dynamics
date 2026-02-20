use numpy::{PyReadonlyArrayDyn, ToPyArray, ndarray::Array1};
use pyo3::prelude::*;

use crate::{
    force::SpatialForce, motion::SpatialMotion, py_force::PySpatialForce, py_vector3d::PyVector3D,
    vector3d::Vector3D, vector6d::Vector6D,
};

/// Spatial motion vector, combining linear and angular velocity components.
///
/// The first three elements represent linear velocity, and the last three represent angular velocity.
#[pyclass(name = "SpatialMotion")]
#[derive(Clone, Debug)]
pub struct PySpatialMotion {
    pub inner: SpatialMotion,
}

#[pymethods]
impl PySpatialMotion {
    #[new]
    /// Creates a new `SpatialMotion` object from a 6D array.
    ///
    /// The first three elements of the array represent the linear velocity, and the last three represent the angular velocity.
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
    /// Creates a new `SpatialMotion` object from separate linear and angular components.
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
    /// Creates a new `SpatialMotion` object with all components set to zero.
    pub fn zero() -> Self {
        PySpatialMotion {
            inner: SpatialMotion::zero(),
        }
    }

    #[getter]
    /// Gets the translation (linear velocity) component of the `SpatialMotion`.
    pub fn translation(&self) -> PyVector3D {
        PyVector3D {
            inner: self.inner.translation(),
        }
    }

    #[getter]
    /// Gets the rotation (angular velocity) component of the `SpatialMotion`.
    pub fn rotation(&self) -> PyVector3D {
        PyVector3D {
            inner: self.inner.rotation(),
        }
    }

    /// Converts the `SpatialMotion` to a NumPy array.
    pub fn to_numpy(&self, py: Python) -> Py<PyAny> {
        Array1::from_iter(self.inner.0.iter().copied())
            .to_pyarray(py)
            .into_any()
            .unbind()
    }

    pub fn __array__(&self, py: Python) -> Py<PyAny> {
        self.to_numpy(py)
    }

    /// Computes the cross product of this `SpatialMotion` with another `SpatialMotion`.
    pub fn cross(&self, other: &PySpatialMotion) -> PySpatialMotion {
        PySpatialMotion {
            inner: SpatialMotion(self.inner.cross(&other.inner).0),
        }
    }

    /// Computes the cross product of this `SpatialMotion` with a `SpatialForce`.
    pub fn cross_force(&self, other: &PySpatialForce) -> PySpatialForce {
        PySpatialForce {
            inner: SpatialForce(self.inner.cross_force(&other.inner).0),
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
