use crate::vector6d::Vector6D;

#[derive(Debug, Clone, PartialEq)]
/// Spatial inertia matrix, represented as a 6x6 matrix.
pub struct SpatialInertia(nalgebra::Matrix6<f64>);

impl SpatialInertia {
    pub fn from_diagonal(diag: &Vector6D) -> Self {
        Self(diag.as_diagonal())
    }
}
