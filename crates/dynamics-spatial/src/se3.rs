//! Special Euclidean group SE(3) implementation.
//!
//! This module provides Rust and Python wrappers for the SE(3) group.

use nalgebra::{IsometryMatrix3, Rotation3, Translation3};
use numpy::{
    PyArrayMethods, PyReadonlyArray1, PyReadonlyArray2, ToPyArray,
    ndarray::{Array1, Array2},
};
use pyo3::{exceptions::PyValueError, prelude::*};

use crate::{motion::SpatialRotation, transform::SpatialTransform, vector3d::Vector3D};

/// SE(3) transformation represented as an isometry matrix.
#[derive(Clone, Debug, Copy, PartialEq, Default)]
pub struct SE3(pub(crate) IsometryMatrix3<f64>);

impl SE3 {
    /// Creates a new SE(3) transformation from a rotation (given as axis-angle) and a translation.
    pub fn new(translation: Vector3D, axis_angle: Vector3D) -> Self {
        let rotation = SpatialRotation::from_axis_angle(&axis_angle, axis_angle.norm());
        SE3::from_parts(
            Vector3D::new(translation.0.x, translation.0.y, translation.0.z),
            rotation,
        )
    }

    /// Creates a new SE(3) transformation from a rotation and a translation.
    pub fn from_parts(translation: Vector3D, rotation: SpatialRotation) -> Self {
        SE3(IsometryMatrix3::from_parts(
            Translation3::from(translation.0),
            rotation.0,
        ))
    }

    /// Creates a new identity SE(3) transformation.
    pub fn identity() -> Self {
        SE3(IsometryMatrix3::identity())
    }

    /// Returns the inverse of the SE(3) transformation.
    pub fn inverse(&self) -> Self {
        SE3(self.0.inverse())
    }

    /// Returns the translation component of the SE(3) transformation.
    pub fn translation(&self) -> Vector3D {
        Vector3D(self.0.translation.vector)
    }

    /// Returns the rotation component of the SE(3) transformation.
    pub fn rotation(&self) -> SpatialRotation {
        SpatialRotation(self.0.rotation)
    }

    /// Computes the action of the SE(3) transformation on a 3D point.
    pub fn action(&self) -> SpatialTransform {
        SpatialTransform::from_se3(self)
    }
}

impl std::ops::Mul for SE3 {
    type Output = SE3;

    fn mul(self, rhs: Self) -> Self::Output {
        SE3(self.0 * rhs.0)
    }
}

impl std::ops::Mul<&SE3> for &SE3 {
    type Output = SE3;

    fn mul(self, rhs: &SE3) -> Self::Output {
        SE3(self.0 * rhs.0)
    }
}

impl std::ops::Mul<SE3> for &SE3 {
    type Output = SE3;

    fn mul(self, rhs: SE3) -> Self::Output {
        SE3(self.0 * rhs.0)
    }
}

impl std::ops::Mul<&SE3> for SE3 {
    type Output = SE3;

    fn mul(self, rhs: &SE3) -> Self::Output {
        SE3(self.0 * rhs.0)
    }
}

/// Python wrapper for the SE(3) group.
#[pyclass(name = "SE3")]
pub struct PySE3 {
    pub inner: SE3,
}

#[pymethods]
impl PySE3 {
    /// Creates a new isometry, that is an element of the special Euclidean group (`SE3`)
    /// with the given rotation and translation.
    ///
    /// # Arguments
    ///
    /// * `rotation` - The rotation matrix (of shape (3, 3)).
    /// * `translation` - The translation vector (of shape (3,)).
    #[new]
    pub fn new(
        rotation: PyReadonlyArray2<f64>,
        translation: PyReadonlyArray1<f64>,
    ) -> PyResult<Self> {
        // TODO: use dynamic arrays and check the shapes
        // Convert the input arrays to vectors
        let rotation = rotation.as_array();
        let translation = translation.as_array();

        // Check the size of the input arrays
        if rotation.shape() != [3, 3] {
            return Err(PyValueError::new_err(format!(
                "Invalid input size. Expected a rotation of shape (3,3), got {:?}",
                rotation.shape()
            )));
        }
        if translation.len() != 3 {
            return Err(PyValueError::new_err(format!(
                "Invalid input size. Expected a translation of shape (3,), got {}.",
                translation.len()
            )));
        }

        // Create the translation and rotation components
        let translation = Vector3D::new(translation[0], translation[1], translation[2]);
        let rotation_matrix =
            nalgebra::Matrix3::from_iterator(rotation.iter().cloned()).transpose();
        if !rotation_matrix.is_orthogonal(1e-6) {
            return Err(PyValueError::new_err(
                "The rotation matrix is not orthogonal.",
            ));
        }
        let rotation = SpatialRotation(Rotation3::from_matrix_unchecked(rotation_matrix));
        let inner = SE3::from_parts(translation, rotation);

        Ok(Self { inner })
    }

    #[pyo3(name = "Identity")]
    #[staticmethod]
    pub fn identity() -> PySE3 {
        let inner = SE3::identity();
        PySE3 { inner }
    }

    #[pyo3(name = "Random")]
    #[staticmethod]
    pub fn random() -> PySE3 {
        let translation = Vector3D::new(
            rand::random::<f64>() * 2.0 - 1.0,
            rand::random::<f64>() * 2.0 - 1.0,
            rand::random::<f64>() * 2.0 - 1.0,
        );
        let axis_angle = Vector3D::new(
            rand::random::<f64>() * 2.0 - 1.0,
            rand::random::<f64>() * 2.0 - 1.0,
            rand::random::<f64>() * 2.0 - 1.0,
        );
        let inner = SE3::new(translation, axis_angle);
        PySE3 { inner }
    }

    #[pyo3(name = "inverse")]
    pub fn inverse(&self) -> PySE3 {
        PySE3 {
            inner: self.inner.inverse(),
        }
    }

    #[getter]
    pub fn get_translation(&self, py: Python) -> Py<PyAny> {
        let translation = self.inner.translation();
        Array1::from_shape_vec(3, translation.as_slice().to_vec())
            .unwrap()
            .to_pyarray(py)
            .into_any()
            .unbind()
    }

    // TODO: implement a TranslationWrapper to allow
    // element-wise operations on the translation vector
    #[setter]
    pub fn set_translation(&mut self, translation: PyReadonlyArray1<f64>) -> PyResult<()> {
        let translation = translation.as_array();

        // Ensure the input translation has the correct shape
        if translation.len() != 3 {
            return Err(PyValueError::new_err(format!(
                "Invalid input size. Expected a translation of shape (3,), got {}.",
                translation.len()
            )));
        }

        // Update the translation component of the inner SE3
        self.inner.0.translation =
            Translation3::new(translation[0], translation[1], translation[2]);
        Ok(())
    }

    #[getter]
    pub fn rotation(&self, py: Python) -> PyResult<Py<PyAny>> {
        let rotation = self.inner.0.rotation;
        Ok(
            Array2::from_shape_vec((3, 3), rotation.matrix().as_slice().to_vec())
                .unwrap()
                .to_pyarray(py)
                .transpose()?
                .into_any()
                .unbind(),
        )
    }

    pub fn copy(&self) -> PySE3 {
        PySE3 { inner: self.inner }
    }

    #[getter]
    pub fn get_homogeneous(&self, py: Python) -> PyResult<Py<PyAny>> {
        let homogeneous = self.inner.0.to_homogeneous();
        Ok(
            Array2::from_shape_vec((4, 4), homogeneous.as_slice().to_vec())
                .unwrap()
                .to_pyarray(py)
                .transpose()?
                .into_any()
                .unbind(),
        )
    }

    fn __mul__(&self, other: &PySE3) -> PySE3 {
        PySE3 {
            inner: self.inner * other.inner,
        }
    }

    fn __repr__(slf: PyRef<'_, Self>) -> String {
        format!("{:?}", slf.inner)
    }
}
