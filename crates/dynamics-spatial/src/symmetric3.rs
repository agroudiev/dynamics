//! Defines **symmetric matrices** of size 3x3 and related operations.

use nalgebra::Matrix3;
use numpy::{PyReadonlyArrayDyn, ToPyArray, ndarray::Array2};
use pyo3::{IntoPyObjectExt, prelude::*};
use std::ops::{Add, Index, Mul, Sub};

use crate::{
    motion::SpatialRotation,
    vector3d::{PyVector3D, Vector3D},
};

/// A symmetric 3x3 matrix.
///
/// The matrix is stored in a compact form, only keeping the unique elements.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Symmetric3 {
    /// The unique elements of the symmetric matrix, stored in the order:
    /// [m11, m22, m33, m12, m13, m23]
    data: [f64; 6],
}

impl Symmetric3 {
    /// Creates a new `Symmetric3` matrix from the given elements.
    ///
    /// # Arguments
    ///
    /// * `m11`, `m22`, `m33` - The diagonal elements.
    /// * `m12`, `m13`, `m23` - The off-diagonal elements.
    #[must_use]
    pub fn new(m11: f64, m22: f64, m33: f64, m12: f64, m13: f64, m23: f64) -> Self {
        Self {
            data: [m11, m22, m33, m12, m13, m23],
        }
    }

    /// Returns the element at the specified row and column.
    ///
    /// # Arguments
    ///
    /// * `row` - The row index (0-based).
    /// * `col` - The column index (0-based).
    ///
    /// # Panics
    ///
    /// Panics if the row or column index is out of bounds.
    #[must_use]
    pub fn get(&self, row: usize, col: usize) -> &f64 {
        match (row, col) {
            (0, 0) => &self.data[0],
            (1, 1) => &self.data[1],
            (2, 2) => &self.data[2],
            (0, 1) | (1, 0) => &self.data[3],
            (0, 2) | (2, 0) => &self.data[4],
            (1, 2) | (2, 1) => &self.data[5],
            _ => panic!("Index out of bounds"),
        }
    }

    /// Returns the zero symmetric matrix.
    #[must_use]
    pub fn zeros() -> Self {
        Self { data: [0.0; 6] }
    }

    /// Returns the identity symmetric matrix.
    #[must_use]
    pub fn identity() -> Self {
        Self {
            data: [1.0, 1.0, 1.0, 0.0, 0.0, 0.0],
        }
    }

    /// Creates a diagonal symmetric matrix from the given diagonal elements.
    ///
    /// # Arguments
    /// * `diag` - A vector containing the diagonal elements [m11, m22, m33].
    #[must_use]
    pub fn from_diagonal(diag: &[f64; 3]) -> Self {
        Self {
            data: [diag[0], diag[1], diag[2], 0.0, 0.0, 0.0],
        }
    }

    /// Convert the symmetric matrix to a full 3x3 matrix.
    #[must_use]
    pub fn matrix(&self) -> Matrix3<f64> {
        Matrix3::new(
            self.data[0],
            self.data[3],
            self.data[4],
            self.data[3],
            self.data[1],
            self.data[5],
            self.data[4],
            self.data[5],
            self.data[2],
        )
    }

    #[must_use]
    pub fn to_numpy(&self, py: Python) -> Py<PyAny> {
        let mat = self.matrix();
        Array2::from_shape_fn((3, 3), |(i, j)| mat[(i, j)])
            .to_pyarray(py)
            .into_any()
            .unbind()
    }

    #[must_use]
    pub fn skew_square(v: Vector3D) -> Symmetric3 {
        let x = v.0[0];
        let y = v.0[1];
        let z = v.0[2];

        Symmetric3::new(
            -y * y - z * z,
            -x * x - z * z,
            -x * x - y * y,
            x * y,
            x * z,
            y * z,
        )
    }

    /// Computes the matrix product $RSR^\top$ where $R$ is a spatial rotation and $S$ is this symmetric matrix.
    ///
    /// # Arguments
    /// * `rotation` - The spatial rotation to apply.
    ///
    /// # Returns
    /// The rotated symmetric matrix.
    #[must_use]
    pub fn rotate(&self, rotation: &SpatialRotation) -> Symmetric3 {
        // TODO: avoid constructing the full matrix
        let r = &rotation.0;
        let s = &self.matrix();
        let rsrt = r * s * r.transpose();
        Symmetric3::new(
            rsrt[(0, 0)],
            rsrt[(1, 1)],
            rsrt[(2, 2)],
            rsrt[(0, 1)],
            rsrt[(0, 2)],
            rsrt[(1, 2)],
        )
    }

    pub fn from_pyarray(array: &PyReadonlyArrayDyn<f64>) -> Result<Self, PyErr> {
        let array = array.as_array();
        if array.shape() != [3, 3] {
            return Err(PyErr::new::<pyo3::exceptions::PyValueError, _>(
                "Input array must be of shape (3, 3)",
            ));
        }

        // Check symmetry
        for i in 0..3 {
            for j in 0..3 {
                if array[[i, j]] != array[[j, i]] {
                    return Err(PyErr::new::<pyo3::exceptions::PyValueError, _>(
                        "Input array must be symmetric",
                    ));
                }
            }
        }

        Ok(Symmetric3::new(
            array[[0, 0]],
            array[[1, 1]],
            array[[2, 2]],
            array[[0, 1]],
            array[[0, 2]],
            array[[1, 2]],
        ))
    }
}

impl Index<(usize, usize)> for Symmetric3 {
    type Output = f64;

    fn index(&self, index: (usize, usize)) -> &Self::Output {
        self.get(index.0, index.1)
    }
}

impl Add<Symmetric3> for Symmetric3 {
    type Output = Symmetric3;

    fn add(self, rhs: Symmetric3) -> Self::Output {
        Symmetric3 {
            data: [
                self.data[0] + rhs.data[0],
                self.data[1] + rhs.data[1],
                self.data[2] + rhs.data[2],
                self.data[3] + rhs.data[3],
                self.data[4] + rhs.data[4],
                self.data[5] + rhs.data[5],
            ],
        }
    }
}

impl Sub<Symmetric3> for Symmetric3 {
    type Output = Symmetric3;

    fn sub(self, rhs: Symmetric3) -> Self::Output {
        Symmetric3 {
            data: [
                self.data[0] - rhs.data[0],
                self.data[1] - rhs.data[1],
                self.data[2] - rhs.data[2],
                self.data[3] - rhs.data[3],
                self.data[4] - rhs.data[4],
                self.data[5] - rhs.data[5],
            ],
        }
    }
}

impl Mul<&Vector3D> for &Symmetric3 {
    type Output = Vector3D;

    fn mul(self, rhs: &Vector3D) -> Self::Output {
        Vector3D::new(
            self[(0, 0)] * rhs.0[0] + self[(0, 1)] * rhs.0[1] + self[(0, 2)] * rhs.0[2],
            self[(1, 0)] * rhs.0[0] + self[(1, 1)] * rhs.0[1] + self[(1, 2)] * rhs.0[2],
            self[(2, 0)] * rhs.0[0] + self[(2, 1)] * rhs.0[1] + self[(2, 2)] * rhs.0[2],
        )
    }
}

impl Mul<f64> for Symmetric3 {
    type Output = Symmetric3;

    fn mul(self, rhs: f64) -> Self::Output {
        Symmetric3 {
            data: [
                self.data[0] * rhs,
                self.data[1] * rhs,
                self.data[2] * rhs,
                self.data[3] * rhs,
                self.data[4] * rhs,
                self.data[5] * rhs,
            ],
        }
    }
}

impl Mul<Symmetric3> for f64 {
    type Output = Symmetric3;

    fn mul(self, rhs: Symmetric3) -> Self::Output {
        Symmetric3 {
            data: [
                rhs.data[0] * self,
                rhs.data[1] * self,
                rhs.data[2] * self,
                rhs.data[3] * self,
                rhs.data[4] * self,
                rhs.data[5] * self,
            ],
        }
    }
}

#[derive(FromPyObject)]
pub enum PySymmetric3Mul {
    Scalar(f64),
    Vector3D(PyVector3D),
}

#[pyclass(name = "Symmetric3")]
#[derive(Debug, Clone, PartialEq)]
pub struct PySymmetric3 {
    inner: Symmetric3,
}

#[pymethods]
impl PySymmetric3 {
    #[new]
    #[must_use]
    pub fn from_elements(m11: f64, m22: f64, m33: f64, m12: f64, m13: f64, m23: f64) -> Self {
        PySymmetric3 {
            inner: Symmetric3::new(m11, m22, m33, m12, m13, m23),
        }
    }

    #[staticmethod]
    #[pyo3(name = "Zero")]
    pub fn zeros() -> Self {
        PySymmetric3 {
            inner: Symmetric3::zeros(),
        }
    }

    #[staticmethod]
    #[pyo3(name = "Identity")]
    pub fn identity() -> Self {
        PySymmetric3 {
            inner: Symmetric3::identity(),
        }
    }

    pub fn __add__(&self, other: &PySymmetric3) -> PySymmetric3 {
        PySymmetric3 {
            inner: self.inner + other.inner,
        }
    }

    pub fn __sub__(&self, other: &PySymmetric3) -> PySymmetric3 {
        PySymmetric3 {
            inner: self.inner - other.inner,
        }
    }

    pub fn __mul__(&self, py: Python, other: PySymmetric3Mul) -> Result<Py<PyAny>, PyErr> {
        match other {
            PySymmetric3Mul::Scalar(scalar) => PySymmetric3 {
                inner: self.inner * scalar,
            }
            .into_py_any(py),
            PySymmetric3Mul::Vector3D(vec) => PyVector3D {
                inner: &self.inner * &vec.inner,
            }
            .into_py_any(py),
        }
    }

    pub fn __rmul__(&self, scalar: f64) -> PySymmetric3 {
        PySymmetric3 {
            inner: self.inner * scalar,
        }
    }

    pub fn to_numpy(&self, py: Python) -> Py<PyAny> {
        self.inner.to_numpy(py)
    }

    pub fn matrix(&self, py: Python) -> Py<PyAny> {
        self.to_numpy(py)
    }

    fn __getitem__(&self, index: (usize, usize)) -> f64 {
        self.inner[index]
    }

    fn __repr__(&self) -> String {
        format!(
            "Symmetric3(\n  [{:.4}, {:.4}, {:.4}],\n  [{:.4}, {:.4}, {:.4}],\n  [{:.4}, {:.4}, {:.4}]\n)",
            self.inner[(0, 0)],
            self.inner[(0, 1)],
            self.inner[(0, 2)],
            self.inner[(1, 0)],
            self.inner[(1, 1)],
            self.inner[(1, 2)],
            self.inner[(2, 0)],
            self.inner[(2, 1)],
            self.inner[(2, 2)],
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_symmetric3_creation() {
        let sym = Symmetric3::new(1.0, 2.0, 3.0, 0.1, 0.2, 0.3);
        assert_eq!(sym[(0, 0)], 1.0);
        assert_eq!(sym[(1, 1)], 2.0);
        assert_eq!(sym[(2, 2)], 3.0);
        assert_eq!(sym[(0, 1)], 0.1);
        assert_eq!(sym[(1, 0)], 0.1);
        assert_eq!(sym[(0, 2)], 0.2);
        assert_eq!(sym[(2, 0)], 0.2);
        assert_eq!(sym[(1, 2)], 0.3);
        assert_eq!(sym[(2, 1)], 0.3);
    }

    #[test]
    fn test_symmetric3_to_matrix() {
        let sym = Symmetric3::new(1.0, 2.0, 3.0, 0.1, 0.2, 0.3);
        let mat = sym.matrix();
        let expected = Matrix3::new(1.0, 0.1, 0.2, 0.1, 2.0, 0.3, 0.2, 0.3, 3.0);
        assert_relative_eq!(mat, expected);
    }

    #[test]
    fn test_symmetric3_mul_vector3d() {
        let sym = Symmetric3::new(1.0, 2.0, 3.0, 0.0, 0.0, 0.0);
        let vec = Vector3D::new(1.0, 2.0, 3.0);
        let result = &sym * &vec;
        let expected = Vector3D::new(1.0, 4.0, 9.0);
        assert_relative_eq!(result.0, expected.0);
    }

    #[test]
    fn test_symmetric3_rotate() {
        let full = Matrix3::new(1.0, 2.0, 3.0, 2.0, 4.0, 5.0, 3.0, 5.0, 6.0);
        assert!(full == full.transpose());

        let sym = Symmetric3::new(
            full[(0, 0)],
            full[(1, 1)],
            full[(2, 2)],
            full[(0, 1)],
            full[(0, 2)],
            full[(1, 2)],
        );

        let rotation = SpatialRotation::from_axis_angle(
            &Vector3D::new(0.0, 0.0, 1.0),
            std::f64::consts::FRAC_PI_2,
        );
        let rotated_sym = sym.rotate(&rotation);

        let expected = rotation.0 * full * rotation.0.transpose();
        let expected_sym = Symmetric3::new(
            expected[(0, 0)],
            expected[(1, 1)],
            expected[(2, 2)],
            expected[(0, 1)],
            expected[(0, 2)],
            expected[(1, 2)],
        );
        assert_relative_eq!(rotated_sym.matrix(), expected_sym.matrix());
    }
}
