use dynamics_spatial::{
    py_force::PySpatialForce, py_motion::PySpatialMotion, py_se3::PySE3, symmetric3::Symmetric3,
    vector3d::Vector3D,
};
use numpy::PyReadonlyArrayDyn;
use pyo3::{exceptions::PyValueError, prelude::*};

use crate::inertia::Inertia;

/// A data structure that contains the information about the inertia of a rigid body (mass, center of mass, and inertia matrix).
#[pyclass(name = "Inertia")]
#[derive(Clone, Debug, Default)]
pub struct PyInertia {
    pub inner: Inertia,
}

#[pymethods]
impl PyInertia {
    /// Creates a new inertia sphere with the given parameters.
    ///
    /// # Arguments
    ///
    /// * `mass` - The mass of the sphere.
    /// * `radius` - The radius of the sphere.
    ///
    /// # Returns
    /// A new [`Inertia`] object representing a sphere.
    #[pyo3(name = "FromSphere")]
    #[staticmethod]
    pub fn from_sphere(mass: f64, radius: f64) -> PyResult<Self> {
        Inertia::from_sphere(mass, radius)
            .map(|inner| PyInertia { inner })
            .map_err(|e| PyValueError::new_err(format!("Failed to create Inertia: {e:?}")))
    }

    /// Creates a new inertia from given mass, center of mass, and inertia matrix.
    ///
    /// # Arguments
    /// * `mass` - The mass of the object.
    /// * `com` - The center of mass of the object, given as a 3D vector (numpy array).
    /// * `inertia` - The rotational inertia matrix of the object at the center of mass, given as a 3x3 symmetric matrix (numpy array).
    #[new]
    pub fn new(
        mass: f64,
        com: PyReadonlyArrayDyn<f64>,
        inertia: PyReadonlyArrayDyn<f64>,
    ) -> PyResult<Self> {
        let com = Vector3D::from_pyarray(&com)?;
        let inertia = Symmetric3::from_pyarray(&inertia)?;
        Ok(PyInertia {
            inner: Inertia::new(mass, com, inertia),
        })
    }

    /// Creates a new [`Inertia`] object with zero mass, zero center of mass, and zero inertia matrix.
    ///
    /// # Returns
    /// A new [`Inertia`] object with all properties set to zero.
    #[staticmethod]
    #[must_use]
    pub fn zeros() -> Self {
        PyInertia {
            inner: Inertia::zeros(),
        }
    }

    #[getter]
    #[must_use]
    /// Returns the mass of the inertia.
    pub fn mass(&self) -> f64 {
        self.inner.mass
    }

    #[getter]
    #[must_use]
    /// Returns the center of mass of the inertia as a numpy array.
    pub fn com(&self, py: Python) -> Py<PyAny> {
        self.inner.com.to_numpy(py)
    }

    /// Returns the lever (center of mass) of the inertia.
    ///
    /// This is an alias for the `com` property.
    #[getter]
    #[must_use]
    pub fn lever(&self, py: Python) -> Py<PyAny> {
        self.inner.com.to_numpy(py)
    }

    #[getter]
    #[must_use]
    /// Returns the inertia matrix of the inertia as a numpy array.
    pub fn inertia(&self, py: Python) -> Py<PyAny> {
        self.inner.inertia.to_numpy(py)
    }

    fn __repr__(slf: PyRef<'_, Self>) -> String {
        format!("{:?}", slf.inner)
    }

    pub fn __add__(&self, other: &PyInertia) -> PyInertia {
        PyInertia {
            inner: self.inner.clone() + other.inner.clone(),
        }
    }

    pub fn __mul__(&self, other: &PySpatialMotion) -> PySpatialForce {
        PySpatialForce {
            inner: &self.inner * &other.inner,
        }
    }

    /// Transforms the inertia to a different frame using the given SE3 transformation.
    pub fn transform_frame(&self, py: Python, other: &PySE3) -> Py<PyAny> {
        let inertia_matrix = self.inner.matrix();
        let transformed = inertia_matrix.transform_frame(&other.inner);
        transformed.to_numpy(py)
    }

    /// Returns the inertia matrix of the inertia as a numpy array.
    pub fn matrix(&self, py: Python) -> Py<PyAny> {
        self.inner.matrix().to_numpy(py)
    }
}
