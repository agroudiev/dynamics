//! Defines spatial **inertia** and related operations.

use nalgebra::Matrix6;

use crate::{motion::SpatialMotion, vector6d::Vector6D};
use std::ops::Mul;

#[derive(Debug, Clone, PartialEq, Default)]
/// Spatial inertia matrix, represented as a 6x6 matrix.
///
/// The spatial inertia matrix is expressed as:
/// $$\begin{bmatrix} mI_3 & -mc_\times \\\\ mc_\times & I_c \end{bmatrix}$$
/// where $m$ is the mass, $I_3$ is the 3x3 identity matrix,
/// $c_\times=\[c\]_\times$ is the skew-symmetric matrix of the center
/// of mass vector, and $I_c$ is the rotational inertia matrix about the center of mass.
pub struct SpatialInertia(Matrix6<f64>);

impl SpatialInertia {
    pub fn new(ixx: f64, ixy: f64, ixz: f64, iyy: f64, iyz: f64, izz: f64) -> Self {
        let mut mat = Matrix6::<f64>::zeros();
        mat[(0, 0)] = ixx;
        mat[(0, 1)] = ixy;
        mat[(0, 2)] = ixz;
        mat[(1, 0)] = ixy;
        mat[(1, 1)] = iyy;
        mat[(1, 2)] = iyz;
        mat[(2, 0)] = ixz;
        mat[(2, 1)] = iyz;
        mat[(2, 2)] = izz;
        Self(mat)
    }

    pub fn from_diagonal(diag: &Vector6D) -> Self {
        Self(diag.as_diagonal())
    }
}

impl Mul<&SpatialMotion> for &SpatialInertia {
    type Output = SpatialMotion;

    fn mul(self, rhs: &SpatialMotion) -> Self::Output {
        SpatialMotion(self.0 * rhs.0)
    }
}
