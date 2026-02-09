//! Defines **3D vectors** and related operations.

use nalgebra::Vector3;
use std::ops::{Add, Mul, Neg, Sub};

#[cfg(feature = "python")]
use numpy::{PyReadonlyArrayDyn, ToPyArray, ndarray::Array1};
#[cfg(feature = "python")]
use pyo3::prelude::*;

#[derive(Debug, Clone, Copy, PartialEq, Default)]
/// A 3D vector, commonly used for positions.
pub struct Vector3D(pub(crate) Vector3<f64>);

impl Vector3D {
    /// Creates a new `Vector3D` with the given x, y, z components.
    #[must_use]
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self(Vector3::new(x, y, z))
    }

    /// Creates a zero vector.
    #[must_use]
    pub fn zeros() -> Self {
        Self(Vector3::zeros())
    }

    #[must_use]
    pub fn as_slice(&self) -> &[f64; 3] {
        self.0.as_slice().try_into().unwrap()
    }

    /// Returns the L2 norm of the vector.
    #[must_use]
    pub fn norm(&self) -> f64 {
        self.0.norm()
    }

    /// Returns the `x` unit vector, that is (1, 0, 0).
    #[must_use]
    pub fn x() -> Self {
        Self(Vector3::x())
    }

    /// Returns the `y` unit vector, that is (0, 1, 0).
    #[must_use]
    pub fn y() -> Self {
        Self(Vector3::y())
    }

    /// Returns the `z` unit vector, that is (0, 0, 1).
    #[must_use]
    pub fn z() -> Self {
        Self(Vector3::z())
    }

    /// Computes the cross product of two 3D vectors.
    #[must_use]
    pub fn cross(&self, other: &Vector3D) -> Vector3D {
        Vector3D(self.0.cross(&other.0))
    }

    #[must_use]
    #[cfg(feature = "python")]
    /// Converts the `Vector3D` to a one-dimensional NumPy array of length 3.
    pub fn to_numpy(&self, py: Python) -> Py<PyAny> {
        Array1::from_iter(self.0.iter().copied())
            .to_pyarray(py)
            .into_any()
            .unbind()
    }

    #[cfg(feature = "python")]
    /// Creates a `Vector3D` from a one-dimensional NumPy array of length 3.
    ///
    /// # Errors
    /// Returns a `PyValueError` if the input array does not have the correct shape
    pub fn from_pyarray(array: &PyReadonlyArrayDyn<f64>) -> Result<Self, PyErr> {
        // FIXME: replace this by a trait implementation
        let array = array.as_array();
        if array.ndim() != 1 || array.len() != 3 {
            return Err(PyErr::new::<pyo3::exceptions::PyValueError, _>(
                "Input array must be one-dimensional with length 3.",
            ));
        }
        Ok(Vector3D(Vector3::new(array[0], array[1], array[2])))
    }

    pub fn dot(&self, other: &Vector3D) -> f64 {
        self.0.dot(&other.0)
    }
}

impl From<&[f64; 3]> for Vector3D {
    fn from(array: &[f64; 3]) -> Self {
        Vector3D(Vector3::new(array[0], array[1], array[2]))
    }
}

impl Add for Vector3D {
    type Output = Vector3D;

    fn add(self, rhs: Self) -> Self::Output {
        Vector3D(self.0 + rhs.0)
    }
}

impl Sub for Vector3D {
    type Output = Vector3D;

    fn sub(self, rhs: Self) -> Self::Output {
        Vector3D(self.0 - rhs.0)
    }
}

impl Mul for Vector3D {
    type Output = Vector3D;

    fn mul(self, rhs: Self) -> Self::Output {
        Vector3D(self.0.component_mul(&rhs.0))
    }
}

impl Mul<f64> for Vector3D {
    type Output = Vector3D;

    fn mul(self, rhs: f64) -> Self::Output {
        Vector3D(self.0 * rhs)
    }
}

impl Mul<f64> for &Vector3D {
    type Output = Vector3D;

    fn mul(self, rhs: f64) -> Self::Output {
        Vector3D(self.0 * rhs)
    }
}

impl Mul<&Vector3D> for f64 {
    type Output = Vector3D;

    fn mul(self, rhs: &Vector3D) -> Self::Output {
        Vector3D(rhs.0 * self)
    }
}

impl Mul<Vector3D> for f64 {
    type Output = Vector3D;

    fn mul(self, rhs: Vector3D) -> Self::Output {
        Vector3D(rhs.0 * self)
    }
}

impl Neg for Vector3D {
    type Output = Vector3D;

    fn neg(self) -> Self::Output {
        Vector3D(-self.0)
    }
}
