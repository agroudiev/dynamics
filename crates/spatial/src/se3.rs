use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};
use pyo3::{exceptions::PyValueError, prelude::*};
use numpy::{PyReadonlyArray1, PyReadonlyArray2};

#[pyclass(name = "SE3")]
pub struct PySE3 {
    pub inner: Isometry3<f64>,
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
        // Convert the input arrays to vectors
        let rotation = rotation.as_array();
        let translation = translation.as_array();

        // Check the size of the input arrays
        if rotation.shape() != [3, 3] {
            return Err(PyValueError::new_err(
                format!("Invalid input size. Expected a rotation of shape (3,3), got {:?}", rotation.shape()),
            ));
        }
        if translation.len() != 3 {
            return Err(PyValueError::new_err(
                format!("Invalid input size. Expected a translation of shape (3,), got {}.",
                translation.len()),
            ));
        }

        // Create the translation and rotation components
        let translation = Translation3::new(translation[0], translation[1], translation[2]);
        let rotation_matrix = nalgebra::Matrix3::from_iterator(rotation.iter().cloned());
        if !rotation_matrix.is_orthogonal(1e-6) {
            return Err(PyValueError::new_err(
                "The rotation matrix is not orthogonal.",
            ));
        }
        let rotation = UnitQuaternion::from_rotation_matrix(&nalgebra::Rotation3::from_matrix_unchecked(rotation_matrix));
        let inner = Isometry3::from_parts(translation, rotation);
        
        Ok(Self { inner })
    }
}

/// The identity element of the special Euclidean group (`SE3`).
/// This is the element that represents no translation or rotation.
#[pyfunction(name = "Identity")]
pub fn identity() -> PySE3 {
    let inner = Isometry3::identity();
    PySE3 { inner }
}