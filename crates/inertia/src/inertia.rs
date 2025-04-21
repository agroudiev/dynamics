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
}

/// A Python wrapper for the `Inertia` struct.
#[pyclass(name = "Inertia")]
pub struct PyInertia {
    pub inner: Inertia,
}

#[pymethods]
impl PyInertia {
    /// Creates a new `Inertia` object with the given parameters.
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
    // TODO: implement this function for the pure Rust version and call it
    pub fn from_sphere(mass: f64, radius: f64) -> PyResult<Self> {
        if mass <= 0.0 {
            return Err(PyValueError::new_err("Mass must be positive."));
        }
        if radius <= 0.0 {
            return Err(PyValueError::new_err("Radius must be positive."));
        }

        let inertia = (2.0 / 5.0) * mass * radius.powi(2);
        let inertia_matrix = Matrix3::new(inertia, 0.0, 0.0, 0.0, inertia, 0.0, 0.0, 0.0, inertia);

        Ok(Self {
            inner: Inertia::new(mass, Vector3::zeros(), inertia_matrix),
        })
    }
}
