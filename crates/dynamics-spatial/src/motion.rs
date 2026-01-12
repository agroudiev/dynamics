//! Defines spatial **motion** and related operations.

use nalgebra::{Matrix6, Rotation3, Vector6};

use crate::{se3::SE3, vector3d::Vector3D, vector6d::Vector6D};
use std::ops::{Add, Mul};

#[derive(Clone, Debug, PartialEq, Default)]
/// Spatial motion vector, combining angular and linear velocity components.
pub struct SpatialMotion(pub(crate) Vector6<f64>);
// TODO: rewrite this with Vector6D to avoid double implementation

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

    /// Extracts the translation (linear velocity) component of the spatial motion.
    pub fn translation(&self) -> Vector3D {
        Vector3D(self.0.fixed_rows::<3>(3).into())
    }

    /// Zero spatial motion (no motion).
    pub fn zero() -> Self {
        Self(Vector6::zeros())
    }

    /// Creates a `SpatialMotion` from angular and linear components.
    ///
    /// # Arguments
    /// * `angular` - The angular velocity component (3D vector).
    /// * `linear` - The linear velocity component (3D vector).
    pub fn from_parts(angular: Vector3D, linear: Vector3D) -> Self {
        let mut v = Vector6::zeros();
        v.fixed_rows_mut::<3>(0).copy_from(&angular.0);
        v.fixed_rows_mut::<3>(3).copy_from(&linear.0);
        Self(v)
    }

    /// Creates a `SpatialMotion` from a 6D vector.
    ///
    /// # Arguments
    /// * `v` - The 6D vector representing spatial motion; the first three elements are angular, the last three are linear.
    pub fn from_vector6d(v: Vector6D) -> Self {
        Self(v.0)
    }

    /// Constructs the cross product matrix for spatial motion vectors.
    ///
    /// # Arguments
    /// * `angular` - The angular velocity component (3D vector).
    /// * `linear` - The linear velocity component (3D vector).
    ///
    /// # Returns
    /// A 6x6 matrix representing the cross product operation.
    fn cross_matrix(angular: Vector3D, linear: Vector3D) -> Matrix6<f64> {
        let mut cross_matrix = Matrix6::zeros();
        let angular_so3 = crate::so3::SO3::from_vector3d(&angular);
        let linear_so3 = crate::so3::SO3::from_vector3d(&linear);
        cross_matrix
            .view_mut((0, 0), (3, 3))
            .copy_from(&angular_so3.0);
        cross_matrix
            .view_mut((0, 3), (3, 3))
            .copy_from(&linear_so3.0);
        cross_matrix
            .view_mut((3, 3), (3, 3))
            .copy_from(&angular_so3.0);
        cross_matrix
    }

    /// Computes the cross product of two spatial motion vectors.
    ///
    /// # Arguments
    /// * `other` - The other spatial motion vector to compute the cross product with.
    ///
    /// # Returns
    /// A new `SpatialMotion` representing the cross product.
    pub fn cross(&self, other: &SpatialMotion) -> SpatialMotion {
        let angular = self.rotation();
        let linear = self.translation();

        let cross_matrix = SpatialMotion::cross_matrix(angular, linear);

        SpatialMotion(cross_matrix * other.0)
    }

    /// Computes the dual cross product of two spatial motion vectors.
    ///
    /// # Arguments
    /// * `other` - The other spatial motion vector to compute the dual cross product with.
    ///
    /// # Returns
    /// A new `SpatialMotion` representing the dual cross product.
    pub fn cross_star(&self, other: &SpatialMotion) -> SpatialMotion {
        let angular = self.rotation();
        let linear = self.translation();

        let cross_matrix = SpatialMotion::cross_matrix(angular, linear);
        let dual_cross_matrix = -cross_matrix.transpose();

        SpatialMotion(dual_cross_matrix * other.0)
    }

    /// Computes the inner product of two spatial motion vectors.
    pub fn inner(&self, other: &SpatialMotion) -> f64 {
        self.0.dot(&other.0)
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
/// Spatial rotation, represented as a 3x3 rotation matrix.
pub struct SpatialRotation(pub(crate) Rotation3<f64>);

impl SpatialRotation {
    /// Creates a new `SpatialRotation` from a 3D axis and an angle (for revolute joints).
    pub fn from_axis_angle(axis: &Vector3D, angle: f64) -> Self {
        let rot = Rotation3::from_axis_angle(&nalgebra::Unit::new_normalize(axis.0), angle);
        Self(rot)
    }

    /// Converts the rotation to an [[[`SE3`]]] with the given translation.
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

impl Mul<&Vector3D> for SpatialRotation {
    type Output = Vector3D;

    fn mul(self, rhs: &Vector3D) -> Self::Output {
        Vector3D(self.0 * rhs.0)
    }
}

#[cfg(test)]
mod tests {
    use crate::so3::SO3;
    use approx::assert_relative_eq;
    use nalgebra::{Matrix3, Matrix6};

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

    #[test]
    fn test_cross() {
        let angular1 = Vector3D::new(1.0, 2.0, 3.0);
        let linear1 = Vector3D::new(4.0, 5.0, 6.0);
        let motion1 = SpatialMotion::from_parts(angular1, linear1);

        let angular2 = Vector3D::new(7.0, 8.0, 9.0);
        let linear2 = Vector3D::new(10.0, 11.0, 12.0);
        let motion2 = SpatialMotion::from_parts(angular2, linear2);

        let mut matrix = Matrix6::zeros();
        let angular_so3 = SO3::from_vector3d(&angular1);
        let linear_so3 = SO3::from_vector3d(&linear1);
        matrix.view_mut((0, 0), (3, 3)).copy_from(&angular_so3.0);
        matrix.view_mut((0, 3), (3, 3)).copy_from(&linear_so3.0);
        matrix.view_mut((3, 3), (3, 3)).copy_from(&angular_so3.0);

        let expected_cross = SpatialMotion(matrix * motion2.0);
        let result_cross = motion1.cross(&motion2);
        let expected_cross_star = SpatialMotion(-matrix.transpose() * motion2.0);
        let result_cross_star = motion1.cross_star(&motion2);

        assert_relative_eq!(result_cross.0, expected_cross.0);
        assert_relative_eq!(result_cross_star.0, expected_cross_star.0);
    }
}
