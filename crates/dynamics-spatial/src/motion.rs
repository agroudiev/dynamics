//! Defines spatial **motion** and related operations.

use nalgebra::{Matrix6, Rotation3, Vector6};

use crate::{
    force::SpatialForce,
    se3::{ActSE3, SE3},
    vector3d::Vector3D,
    vector6d::Vector6D,
};
use std::{
    fmt::Display,
    ops::{Add, AddAssign, Mul},
};

#[derive(Clone, Debug, PartialEq, Default)]
/// Spatial motion vector, combining linear and angular velocity components.
///
/// The first three elements represent linear velocity, and the last three represent angular velocity.
pub struct SpatialMotion(pub(crate) Vector6<f64>);
// TODO: rewrite this with Vector6D to avoid double implementation

impl SpatialMotion {
    /// Creates a new `SpatialMotion` from a 3D rotational axis (for revolute and continuous joints).
    #[must_use]
    pub fn from_rotational_axis(axis: &Vector3D) -> Self {
        let mut v = Vector6::zeros();
        v.fixed_rows_mut::<3>(3).copy_from(&axis.0);
        Self(v)
    }

    /// Creates a new `SpatialMotion` from a 3D translational axis (for prismatic joints).
    #[must_use]
    pub fn from_translational_axis(axis: &Vector3D) -> Self {
        let mut v = Vector6::zeros();
        v.fixed_rows_mut::<3>(0).copy_from(&axis.0);
        Self(v)
    }

    /// Extracts the rotation (angular velocity) component of the spatial motion.
    #[must_use]
    pub fn rotation(&self) -> Vector3D {
        Vector3D(self.0.fixed_rows::<3>(3).into())
    }

    /// Extracts the translation (linear velocity) component of the spatial motion.
    #[must_use]
    pub fn translation(&self) -> Vector3D {
        Vector3D(self.0.fixed_rows::<3>(0).into())
    }

    /// Zero spatial motion (no motion).
    #[must_use]
    pub fn zero() -> Self {
        Self(Vector6::zeros())
    }

    /// Creates a `SpatialMotion` from linear and angular components.
    ///
    /// # Arguments
    /// * `linear` - The linear velocity component (3D vector).
    /// * `angular` - The angular velocity component (3D vector).
    #[must_use]
    pub fn from_parts(linear: Vector3D, angular: Vector3D) -> Self {
        let mut v = Vector6::zeros();
        v.fixed_rows_mut::<3>(0).copy_from(&linear.0);
        v.fixed_rows_mut::<3>(3).copy_from(&angular.0);
        Self(v)
    }

    /// Creates a `SpatialMotion` from a 6D vector.
    ///
    /// # Arguments
    /// * `v` - The 6D vector representing spatial motion; the first three elements are linear, the last three are angular.
    #[must_use]
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
    #[must_use]
    pub fn cross(&self, other: &SpatialMotion) -> SpatialMotion {
        let angular_1 = self.rotation();
        let linear_1 = self.translation();

        let angular_2 = other.rotation();
        let linear_2 = other.translation();

        let cross_linear = angular_1.cross(&linear_2) + linear_1.cross(&angular_2);
        let cross_angular = angular_1.cross(&angular_2);

        SpatialMotion::from_parts(cross_linear, cross_angular)
    }

    /// Computes the cross product of two spatial motion vectors.
    ///
    /// # Arguments
    /// * `other` - The other spatial motion vector to compute the cross product with.
    ///
    /// # Returns
    /// A new `SpatialMotion` representing the cross product.
    #[must_use]
    pub fn cross_force(&self, other: &SpatialForce) -> SpatialForce {
        let m_linear = self.translation();
        let m_angular = self.rotation();

        let f_linear = other.translation();
        let f_angular = other.rotation();

        let cross_linear = m_angular.cross(&f_linear);
        let cross_angular = m_angular.cross(&f_angular) + m_linear.cross(&f_linear);

        SpatialForce::from_parts(cross_linear, cross_angular)
    }

    /// Computes the dual cross product of two spatial motion vectors.
    ///
    /// # Arguments
    /// * `other` - The other spatial motion vector to compute the dual cross product with.
    ///
    /// # Returns
    /// A new `SpatialMotion` representing the dual cross product.
    #[must_use]
    pub fn cross_star(&self, other: &SpatialMotion) -> SpatialMotion {
        let angular = self.rotation();
        let linear = self.translation();

        let cross_matrix = SpatialMotion::cross_matrix(angular, linear);
        let dual_cross_matrix = -cross_matrix.transpose();

        SpatialMotion(dual_cross_matrix * other.0)
    }

    /// Computes the inner product of two spatial motion vectors.
    #[must_use]
    pub fn inner(&self, other: &SpatialMotion) -> f64 {
        self.0.dot(&other.0)
    }

    /// Returns the spatial motion as a slice of 6 elements (linear followed by angular).
    pub fn as_slice(&self) -> &[f64; 6] {
        self.0
            .as_slice()
            .try_into()
            .expect("Vector6 should have exactly 6 elements")
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

impl AddAssign for SpatialMotion {
    fn add_assign(&mut self, rhs: Self) {
        self.0 += rhs.0;
    }
}

impl AddAssign<&SpatialMotion> for SpatialMotion {
    fn add_assign(&mut self, rhs: &Self) {
        self.0 += rhs.0;
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

impl Mul<SpatialMotion> for f64 {
    type Output = SpatialMotion;

    fn mul(self, rhs: SpatialMotion) -> Self::Output {
        SpatialMotion(rhs.0 * self)
    }
}

impl ActSE3 for SpatialMotion {
    fn act(&self, se3: &SE3) -> Self {
        let angular = se3.rotation() * &self.rotation();
        let linear = se3.rotation() * &self.translation() + se3.translation().cross(&angular);
        SpatialMotion::from_parts(linear, angular)
    }

    fn act_inv(&self, se3: &SE3) -> Self {
        let angular = se3.rotation().transpose() * &self.rotation();
        let linear = se3.rotation().transpose()
            * &(self.translation() - se3.translation().cross(&self.rotation()));
        SpatialMotion::from_parts(linear, angular)
    }
}

impl Display for SpatialMotion {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "SpatialMotion(linear: [{:.4}, {:.4}, {:.4}], angular: [{:.4}, {:.4}, {:.4}])",
            self.translation().0[0],
            self.translation().0[1],
            self.translation().0[2],
            self.rotation().0[0],
            self.rotation().0[1],
            self.rotation().0[2],
        )
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
/// Spatial rotation, represented as a 3x3 rotation matrix.
pub struct SpatialRotation(pub(crate) Rotation3<f64>);

impl SpatialRotation {
    /// Creates a new `SpatialRotation` from a 3D axis and an angle (for revolute joints).
    #[must_use]
    pub fn from_axis_angle(axis: &Vector3D, angle: f64) -> Self {
        let rot = Rotation3::from_axis_angle(&nalgebra::Unit::new_normalize(axis.0), angle);
        Self(rot)
    }

    /// Converts the rotation to an [[[`SE3`]]] with the given translation.
    #[must_use]
    pub fn to_se3(&self, translation: &Vector3D) -> SE3 {
        SE3::from_parts(*translation, *self)
    }

    /// Returns the identity rotation.
    #[must_use]
    pub fn identity() -> Self {
        Self(Rotation3::identity())
    }

    /// Returns the angle of rotation in radians.
    #[must_use]
    pub fn angle(&self) -> f64 {
        self.0.angle()
    }

    #[must_use]
    pub fn from_euler_angles(roll: f64, pitch: f64, yaw: f64) -> Self {
        Self(Rotation3::from_euler_angles(roll, pitch, yaw))
    }

    /// Returns the transpose of the rotation matrix.
    pub fn transpose(&self) -> Self {
        Self(self.0.transpose())
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
        let motion1 = SpatialMotion::from_parts(linear1, angular1);

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
