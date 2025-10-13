use nalgebra::{Rotation3, Translation, Vector6};

use crate::vector3d::Vector3D;

#[derive(Clone, Debug, PartialEq)]
pub struct SpatialMotion(pub(crate) Vector6<f64>);

impl SpatialMotion {
    /// Creates a new `SpatialMotion` from a 3D axis (for revolute joints).
    pub fn from_axis(axis: &Vector3D) -> Self {
        let mut v = Vector6::zeros();
        v.fixed_rows_mut::<3>(0).copy_from(&axis.0);
        Self(v)
    }

    pub fn rotation(&self) -> Vector3D {
        Vector3D(self.0.fixed_rows::<3>(0).into())
    }
}

pub struct SpatialRotation(pub(crate) Rotation3<f64>);

impl SpatialRotation {
    /// Creates a new `SpatialRotation` from a 3D axis and an angle (for revolute joints).
    pub fn from_axis_angle(axis: &Vector3D, angle: f64) -> Self {
        let rot = Rotation3::from_axis_angle(&nalgebra::Unit::new_normalize(axis.0), angle);
        Self(rot)
    }

    pub fn to_se3(&self, translation: &Vector3D) -> crate::se3::SE3 {
        crate::se3::SE3::from_parts(
            Translation {
                vector: translation.0,
            },
            self.0,
        )
    }
}
