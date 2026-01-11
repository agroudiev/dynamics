//! Structures to represent the inertia of a rigid body.

use dynamics_spatial::{
    inertia::SpatialInertia, motion::SpatialMotion, vector3d::Vector3D, vector6d::Vector6D,
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
    /// The inertia matrix of the object.
    pub inertia: SpatialInertia, // TODO: change to I_c
}

impl Inertia {
    /// Creates a new `Inertia` object with the given parameters.
    ///
    /// # Arguments
    ///
    /// * `mass` - The mass of the object.
    /// * `com` - The center of mass of the object (3D vector).
    /// * `inertia` - The inertia matrix of the object (3x3 matrix).
    pub fn new(mass: f64, com: Vector3D, inertia: SpatialInertia) -> Self {
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
            inertia: SpatialInertia::zeros(),
        }
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
        if mass <= 0.0 {
            return Err(InertiaError::InvalidParameter("mass".to_string()));
        }
        if radius <= 0.0 {
            return Err(InertiaError::InvalidParameter("radius".to_string()));
        }

        let inertia = (2.0 / 5.0) * mass * radius.powi(2);
        let diag = Vector6D::new(inertia, inertia, inertia, mass, mass, mass);
        let inertia_matrix = SpatialInertia::from_diagonal(&diag);

        Ok(Self::new(mass, Vector3D::zeros(), inertia_matrix))
    }
}

impl Mul<&SpatialMotion> for &Inertia {
    type Output = SpatialMotion;

    fn mul(self, rhs: &SpatialMotion) -> Self::Output {
        &self.inertia * rhs
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

    // TODO: com getter

    // TODO: inertia getter

    fn __repr__(slf: PyRef<'_, Self>) -> String {
        format!("{:?}", slf.inner)
    }
}
