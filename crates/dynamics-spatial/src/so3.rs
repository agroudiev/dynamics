//! Defines **special orthogonal group** SO(3) and related operations.

use crate::vector3d::Vector3D;
use nalgebra::Matrix3;

pub struct SO3(pub(crate) Matrix3<f64>);

impl SO3 {
    /// Returns the identity rotation.
    #[must_use]
    pub fn identity() -> Self {
        Self(Matrix3::identity())
    }

    #[must_use]
    pub fn from_vector3d(vec: &Vector3D) -> Self {
        let v = vec.as_slice();
        Self(Matrix3::new(
            0.0, -v[2], v[1], v[2], 0.0, -v[0], -v[1], v[0], 0.0,
        ))
    }
}
