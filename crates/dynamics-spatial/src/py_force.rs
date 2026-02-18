use numpy::{PyReadonlyArrayDyn, ToPyArray, ndarray::Array1};
use pyo3::prelude::*;

use crate::{force::SpatialForce, py_vector3d::PyVector3D, vector3d::Vector3D, vector6d::Vector6D};

/// Spatial force vector, combining torque and force components.
///
/// A spatial force is represented as a 6-dimensional vector,
/// which can be decomposed into [f τ] where f is the translational force and τ is the rotational torque.
#[pyclass(name = "SpatialForce")]
#[derive(Clone, Debug)]
pub struct PySpatialForce {
    pub inner: SpatialForce,
}

#[pymethods]
impl PySpatialForce {
    #[new]
    /// Creates a new `SpatialForce` from a 6-dimensional array.
    /// The first three elements represent the linear force, and the last three represent the angular torque.
    pub fn new(array: PyReadonlyArrayDyn<f64>) -> PyResult<Self> {
        let array = array.as_array();
        if array.ndim() != 1 || array.len() != 6 {
            return Err(PyErr::new::<pyo3::exceptions::PyValueError, _>(
                "Input array must be one-dimensional with length 6.",
            ));
        }
        Ok(PySpatialForce {
            inner: SpatialForce(
                Vector6D::new(array[0], array[1], array[2], array[3], array[4], array[5]).0,
            ),
        })
    }

    #[staticmethod]
    /// Creates a new `SpatialForce` from separate linear and angular components.
    pub fn from_vectors(linear: PyVector3D, angular: PyVector3D) -> Self {
        PySpatialForce {
            inner: SpatialForce::from_parts(linear.inner, angular.inner),
        }
    }

    #[staticmethod]
    /// Creates a new `SpatialForce` object.
    /// # Arguments
    /// * `angular` - The angular component as a list of 3 floats.
    /// * `linear` - The linear component as a list of 3 floats.
    /// # Returns
    /// A new `SpatialForce` object.
    pub fn from_parts(
        angular: PyReadonlyArrayDyn<f64>,
        linear: PyReadonlyArrayDyn<f64>,
    ) -> PyResult<Self> {
        Ok(PySpatialForce {
            inner: SpatialForce::from_parts(
                Vector3D::from_pyarray(&angular)?,
                Vector3D::from_pyarray(&linear)?,
            ),
        })
    }

    #[staticmethod]
    /// Creates a new `SpatialForce` object with zero linear and angular components.
    pub fn zero() -> Self {
        PySpatialForce {
            inner: SpatialForce::zero(),
        }
    }

    #[getter]
    /// Returns the linear component of the spatial force as a `Vector3D`.
    pub fn translation(&self) -> PyVector3D {
        PyVector3D {
            inner: self.inner.translation(),
        }
    }

    #[getter]
    /// Returns the angular component of the spatial force as a `Vector3D`.
    pub fn rotation(&self) -> PyVector3D {
        PyVector3D {
            inner: self.inner.rotation(),
        }
    }

    /// Converts the `SpatialForce` to a NumPy array.
    pub fn to_numpy(&self, py: Python) -> Py<PyAny> {
        Array1::from_iter(self.inner.0.iter().copied())
            .to_pyarray(py)
            .into_any()
            .unbind()
    }

    /// Computes the cross product of this spatial force with another spatial force.
    pub fn cross(&self, other: &PySpatialForce) -> PySpatialForce {
        PySpatialForce {
            inner: SpatialForce(self.inner.cross(&other.inner).0),
        }
    }

    pub fn __repr__(&self) -> PyResult<String> {
        Ok(format!("{:?}", self.inner))
    }

    pub fn __add__(&self, other: &PySpatialForce) -> PySpatialForce {
        PySpatialForce {
            inner: SpatialForce(self.inner.0 + other.inner.0),
        }
    }

    #[getter]
    /// Returns the linear component of the spatial force as a `Vector3D`.
    pub fn get_linear(&self) -> PyVector3D {
        PyVector3D {
            inner: self.inner.translation(),
        }
    }

    #[getter]
    /// Returns the angular component of the spatial force as a `Vector3D`.
    pub fn get_angular(&self) -> PyVector3D {
        PyVector3D {
            inner: self.inner.rotation(),
        }
    }
}
