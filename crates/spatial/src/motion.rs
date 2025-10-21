use nalgebra::{Rotation3, Vector6};

use crate::{se3::SE3, vector3d::Vector3D};
use std::ops::{Add, Mul};

#[derive(Clone, Debug, PartialEq)]
/// Spatial motion vector, combining angular and linear velocity components.
pub struct SpatialMotion(pub(crate) Vector6<f64>);

impl SpatialMotion {
    /// Creates a new `SpatialMotion` from a 3D axis (for revolute joints).
    pub fn from_axis(axis: &Vector3D) -> Self {
        let mut v = Vector6::zeros();
        v.fixed_rows_mut::<3>(0).copy_from(&axis.0);
        Self(v)
    }

    /// Extracts the rotation (angular velocity) component of the spatial motion.
    pub fn rotation(&self) -> Vector3D {
        Vector3D(self.0.fixed_rows::<3>(0).into())
    }

    /// Zero spatial motion (no motion).
    pub fn zero() -> Self {
        Self(Vector6::zeros())
    }

    /// Creates a `SpatialMotion` from angular and linear components.
    pub fn from_parts(angular: Vector3D, linear: Vector3D) -> Self {
        let mut v = Vector6::zeros();
        v.fixed_rows_mut::<3>(0).copy_from(&angular.0);
        v.fixed_rows_mut::<3>(3).copy_from(&linear.0);
        Self(v)
    }

    /// Computes the cross product of two spatial motion vectors.
    pub fn cross(&self, other: &SpatialMotion) -> SpatialMotion {
        SpatialMotion(self.0.cross(&other.0))
    }
}

impl Add for SpatialMotion {
    type Output = SpatialMotion;

    fn add(self, rhs: Self) -> Self::Output {
        SpatialMotion(self.0 + rhs.0)
    }
}

impl Add<&SpatialMotion> for SpatialMotion {
    type Output = SpatialMotion;

    fn add(self, rhs: &Self) -> Self::Output {
        SpatialMotion(self.0 + rhs.0)
    }
}

impl Mul<f64> for SpatialMotion {
    type Output = SpatialMotion;

    fn mul(self, rhs: f64) -> Self::Output {
        SpatialMotion(self.0 * rhs)
    }
}

impl Mul<f64> for &SpatialMotion {
    type Output = SpatialMotion;

    fn mul(self, rhs: f64) -> Self::Output {
        SpatialMotion(self.0 * rhs)
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SpatialRotation(pub(crate) Rotation3<f64>);

impl SpatialRotation {
    /// Creates a new `SpatialRotation` from a 3D axis and an angle (for revolute joints).
    pub fn from_axis_angle(axis: &Vector3D, angle: f64) -> Self {
        let rot = Rotation3::from_axis_angle(&nalgebra::Unit::new_normalize(axis.0), angle);
        Self(rot)
    }

    /// Converts the rotation to an `SE3` with the given translation.
    pub fn to_se3(&self, translation: &Vector3D) -> SE3 {
        SE3::from_parts(*translation, *self)
    }

    /// Returns the identity rotation.
    pub fn identity() -> Self {
        Self(Rotation3::identity())
    }

    /// Returns the angle of rotation in radians.
    pub fn angle(&self) -> f64 {
        self.0.angle()
    }

    pub fn from_euler_angles(roll: f64, pitch: f64, yaw: f64) -> Self {
        Self(Rotation3::from_euler_angles(roll, pitch, yaw))
    }
}

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;
    use nalgebra::Matrix3;

    use super::*;

    #[test]
    fn test_spatial_motion_zero() {
        let zero = SpatialMotion::zero();
        assert_eq!(zero.0, Vector6::zeros());
    }

    #[test]
    fn test_spatial_rotation_identity() {
        let identity = SpatialRotation::identity();
        assert_eq!(identity.0, Rotation3::identity());
    }

    #[test]
    fn test_spatial_rotation_pi_2() {
        let z = Vector3D::new(0.0, 0.0, 1.0);
        let rotation = SpatialRotation::from_axis_angle(&z, std::f64::consts::PI / 2.0);
        let expected = Matrix3::new(0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0);
        assert_relative_eq!(rotation.0.matrix(), &expected);
    }
}
