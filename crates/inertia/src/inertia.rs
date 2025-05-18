//! Structures to represent the inertia of a rigid body.

use nalgebra::{Matrix3, Vector3};
use pyo3::{exceptions::PyValueError, prelude::*};

/// A data structure that contains the information about the inertia of a rigid body (mass, center of mass, and inertia matrix).
#[derive(Clone, Debug)]
pub struct Inertia {
    /// The mass of the object.
    pub mass: f64,
    /// The center of mass of the object.
    pub com: Vector3<f64>,
    /// The inertia matrix of the object.
    pub inertia: Matrix3<f64>,
}

impl Inertia {
    /// Creates a new `Inertia` object with the given parameters.
    ///
    /// # Arguments
    ///
    /// * `mass` - The mass of the object.
    /// * `com` - The center of mass of the object (3D vector).
    /// * `inertia` - The inertia matrix of the object (3x3 matrix).
    pub fn new(mass: f64, com: Vector3<f64>, inertia: Matrix3<f64>) -> Self {
        Self { mass, com, inertia }
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
        let inertia_matrix = Matrix3::new(inertia, 0.0, 0.0, 0.0, inertia, 0.0, 0.0, 0.0, inertia);

        Ok(Self::new(mass, Vector3::zeros(), inertia_matrix))
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
}
