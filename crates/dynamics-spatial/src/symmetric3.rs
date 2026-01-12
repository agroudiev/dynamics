//! Defines **symmetric matrices** of size 3x3 and related operations.

use nalgebra::Matrix3;
use numpy::{ToPyArray, ndarray::Array2};
use pyo3::prelude::*;
use std::ops::{Index, Mul};

use crate::vector3d::Vector3D;

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
    pub fn zeros() -> Self {
        Self { data: [0.0; 6] }
    }

    /// Returns the identity symmetric matrix.
    pub fn identity() -> Self {
        Self {
            data: [1.0, 1.0, 1.0, 0.0, 0.0, 0.0],
        }
    }

    /// Creates a diagonal symmetric matrix from the given diagonal elements.
    ///
    /// # Arguments
    /// * `diag` - A vector containing the diagonal elements [m11, m22, m33].
    pub fn from_diagonal(diag: &[f64; 3]) -> Self {
        Self {
            data: [diag[0], diag[1], diag[2], 0.0, 0.0, 0.0],
        }
    }

    /// Convert the symmetric matrix to a full 3x3 matrix.
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

    pub fn to_numpy(&self, py: Python) -> Py<PyAny> {
        let mat = self.matrix();
        Array2::from_shape_fn((3, 3), |(i, j)| mat[(i, j)])
            .to_pyarray(py)
            .into_any()
            .unbind()
    }
}

impl Index<(usize, usize)> for Symmetric3 {
    type Output = f64;

    fn index(&self, index: (usize, usize)) -> &Self::Output {
        self.get(index.0, index.1)
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
}
