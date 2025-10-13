use crate::vector6d::Vector6D;

#[derive(Debug, Clone, PartialEq)]
pub struct SpatialInertia(nalgebra::Matrix6<f64>);

impl SpatialInertia {
    pub fn from_diagonal(diag: &Vector6D) -> Self {
        Self(diag.as_diagonal())
    }
}
