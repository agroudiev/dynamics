//! Structures to represent the inertia of a rigid body.

use dynamics_spatial::{
    force::SpatialForce, motion::SpatialMotion, symmetric3::Symmetric3, vector3d::Vector3D,
};
use pyo3::{exceptions::PyValueError, prelude::*};
use std::ops::Mul;

/// A data structure that contains the information about the inertia of a rigid body (mass, center of mass, and inertia matrix).
#[derive(Clone, Debug, Default)]
pub struct Inertia {
    /// The mass of the object.
    pub mass: f64,
    /// The center of mass of the object.
    pub com: Vector3D,
    /// Rotational inertia matrix at the center of mass.
    pub inertia: Symmetric3,
}

impl Inertia {
    /// Creates a new `Inertia` object with the given parameters.
    ///
    /// # Arguments
    ///
    /// * `mass` - The mass of the object.
    /// * `com` - The center of mass of the object.
    /// * `inertia` - The rotational inertia matrix of the object at the center of mass.
    pub fn new(mass: f64, com: Vector3D, inertia: Symmetric3) -> Self {
        Self { mass, com, inertia }
    }

    /// Creates a new `Inertia` object with zero mass, zero center of mass, and zero inertia matrix.
    ///
    /// # Returns
    /// A new `Inertia` object with all properties set to zero.
    pub fn zeros() -> Self {
        Self {
            mass: 0.0,
            com: Vector3D::zeros(),
            inertia: Symmetric3::zeros(),
        }
    }

    /// Creates a new `Inertia` object representing an ellipsoid with the given mass and semi-axis lengths.
    ///
    /// # Arguments
    /// * `mass` - The mass of the ellipsoid.
    /// * `x` - The semi-axis length along the x-axis.
    /// * `y` - The semi-axis length along the y-axis.
    /// * `z` - The semi-axis length along the z-axis.
    ///
    /// # Returns
    /// A new `Inertia` object representing an ellipsoid.
    pub fn from_ellipsoid(mass: f64, x: f64, y: f64, z: f64) -> Result<Self, InertiaError> {
        if mass <= 0.0 {
            return Err(InertiaError::InvalidParameter("mass".to_string()));
        }
        if x <= 0.0 {
            return Err(InertiaError::InvalidParameter("x".to_string()));
        }
        if y <= 0.0 {
            return Err(InertiaError::InvalidParameter("y".to_string()));
        }
        if z <= 0.0 {
            return Err(InertiaError::InvalidParameter("z".to_string()));
        }

        let a = mass * (y.powi(2) + z.powi(2)) / 5.0;
        let b = mass * (x.powi(2) + z.powi(2)) / 5.0;
        let c = mass * (x.powi(2) + y.powi(2)) / 5.0;
        let inertia_matrix = Symmetric3::new(a, 0.0, b, 0.0, 0.0, c);
        Ok(Self::new(mass, Vector3D::zeros(), inertia_matrix))
    }

    /// Creates a new `Inertia` object representing a sphere with the given mass and radius.
    ///
    /// # Arguments
    ///
    /// * `mass` - The mass of the sphere.
    /// * `radius` - The radius of the sphere.
    ///
    /// # Returns
    /// A new `Inertia` object representing a sphere.
    pub fn from_sphere(mass: f64, radius: f64) -> Result<Self, InertiaError> {
        Inertia::from_ellipsoid(mass, radius, radius, radius)
    }
}

impl Mul<&SpatialMotion> for &Inertia {
    type Output = SpatialForce;

    fn mul(self, rhs: &SpatialMotion) -> Self::Output {
        let linear = self.mass * (rhs.translation() - self.com.cross(&rhs.rotation()));
        let angular = &self.inertia * &rhs.rotation() + self.com.cross(&linear);
        SpatialForce::from_components(angular, linear)
    }
}

/// An error type for the `Inertia` struct.
pub enum InertiaError {
    InvalidParameter(String),
}

impl std::fmt::Display for InertiaError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            InertiaError::InvalidParameter(param) => {
                write!(f, "Invalid parameter: '{}' must be positive.", param)
            }
        }
    }
}

impl std::fmt::Debug for InertiaError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "InertiaError: {}", self)
    }
}

/// A Python wrapper for the `Inertia` struct.
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
    /// A new `Inertia` object representing a sphere.
    #[pyo3(name = "FromSphere")]
    #[staticmethod]
    pub fn from_sphere(mass: f64, radius: f64) -> PyResult<Self> {
        Inertia::from_sphere(mass, radius)
            .map(|inner| PyInertia { inner })
            .map_err(|e| PyValueError::new_err(format!("Failed to create Inertia: {:?}", e)))
    }

    /// Creates a new `Inertia` object with zero mass, zero center of mass, and zero inertia matrix.
    ///
    /// # Returns
    /// A new `Inertia` object with all properties set to zero.
    #[staticmethod]
    pub fn zeros() -> Self {
        PyInertia {
            inner: Inertia::zeros(),
        }
    }

    #[getter]
    pub fn mass(&self) -> f64 {
        self.inner.mass
    }

    #[getter]
    pub fn com(&self, py: Python) -> Py<PyAny> {
        self.inner.com.to_numpy(py)
    }

    /// Returns the lever (center of mass) of the inertia.
    ///
    /// This is an alias for the `com` property.
    #[getter]
    pub fn lever(&self, py: Python) -> Py<PyAny> {
        self.inner.com.to_numpy(py)
    }

    #[getter]
    pub fn inertia(&self, py: Python) -> Py<PyAny> {
        self.inner.inertia.to_numpy(py)
    }

    fn __repr__(slf: PyRef<'_, Self>) -> String {
        format!("{:?}", slf.inner)
    }
}
