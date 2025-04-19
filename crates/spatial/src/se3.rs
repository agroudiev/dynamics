use nalgebra::{IsometryMatrix3, Translation3, Vector3};
use numpy::{
    PyArrayMethods, PyReadonlyArray1, PyReadonlyArray2, ToPyArray,
    ndarray::{Array1, Array2},
};
use pyo3::{exceptions::PyValueError, prelude::*};

#[pyclass(name = "SE3")]
pub struct PySE3 {
    pub inner: IsometryMatrix3<f64>,
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
        let translation = Translation3::new(translation[0], translation[1], translation[2]);
        let rotation_matrix = nalgebra::Matrix3::from_iterator(rotation.iter().cloned());
        if !rotation_matrix.is_orthogonal(1e-6) {
            return Err(PyValueError::new_err(
                "The rotation matrix is not orthogonal.",
            ));
        }
        let rotation = nalgebra::Rotation3::from_matrix_unchecked(rotation_matrix);
        let inner = IsometryMatrix3::from_parts(translation, rotation);

        Ok(Self { inner })
    }

    #[pyo3(name = "Identity")]
    #[staticmethod]
    pub fn identity() -> PySE3 {
        let inner = IsometryMatrix3::identity();
        PySE3 { inner }
    }

    #[pyo3(name = "Random")]
    #[staticmethod]
    pub fn random() -> PySE3 {
        let translation = Vector3::new(
            rand::random::<f64>() * 2.0 - 1.0,
            rand::random::<f64>() * 2.0 - 1.0,
            rand::random::<f64>() * 2.0 - 1.0,
        );
        let axis_angle = nalgebra::Vector3::new(
            rand::random::<f64>() * 2.0 - 1.0,
            rand::random::<f64>() * 2.0 - 1.0,
            rand::random::<f64>() * 2.0 - 1.0,
        );
        let inner = IsometryMatrix3::new(translation, axis_angle);
        PySE3 { inner }
    }

    #[pyo3(name = "inverse")]
    pub fn inverse(&self) -> PySE3 {
        PySE3 {
            inner: self.inner.inverse(),
        }
    }

    #[getter]
    pub fn translation(&self, py: Python) -> Py<PyAny> {
        let translation = self.inner.translation.vector;
        Array1::from_shape_vec(3, translation.as_slice().to_vec())
            .unwrap()
            .to_pyarray(py)
            .into_any()
            .unbind()
    }

    #[getter]
    pub fn rotation(&self, py: Python) -> PyResult<Py<PyAny>> {
        let rotation = self.inner.rotation;
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
        PySE3 {
            inner: self.inner,
        }
    }

    #[getter]
    pub fn get_homogeneous(&self, py: Python) -> PyResult<Py<PyAny>> {
        let homogeneous = self.inner.to_homogeneous();
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
}
