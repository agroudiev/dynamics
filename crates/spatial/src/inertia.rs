use crate::{motion::SpatialMotion, vector6d::Vector6D};
use std::ops::Mul;

#[derive(Debug, Clone, PartialEq)]
/// Spatial inertia matrix, represented as a 6x6 matrix.
pub struct SpatialInertia(nalgebra::Matrix6<f64>);

impl SpatialInertia {
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
