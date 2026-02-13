//! Defines spatial **forces** and related operations.

use std::ops::{Add, AddAssign, Sub, SubAssign};

use nalgebra::{Matrix6, Vector6};

use crate::{
    se3::{ActSE3, SE3},
    vector3d::Vector3D,
    vector6d::Vector6D,
};

/// Spatial force vector, combining torque and force components.
///
/// A spatial force is represented as a 6-dimensional vector,
/// which can be decomposed into $\begin{bmatrix} f & \tau \end{bmatrix}$,
/// where $f$ is the force (translational component) and $\tau$ is the torque (rotational component).
#[derive(Debug, Clone, PartialEq)]
pub struct SpatialForce(pub Vector6<f64>);

impl SpatialForce {
    /// Creates a new [`SpatialForce`] from the given force and torque components.
    ///
    /// # Arguments
    ///
    /// * `force` - The force component of the spatial force.
    /// * `torque` - The torque component of the spatial force.
    ///
    /// # Returns
    /// A new [`SpatialForce`] object combining the given torque and force.
    pub fn from_parts(force: Vector3D, torque: Vector3D) -> Self {
        let mut data = Vector6::zeros();
        data.fixed_rows_mut::<3>(0).copy_from(&force.0);
        data.fixed_rows_mut::<3>(3).copy_from(&torque.0);
        Self(data)
    }

    pub fn zero() -> Self {
        Self(Vector6::zeros())
    }

    pub fn from_vector6(vector: Vector6<f64>) -> Self {
        Self(vector)
    }

    pub fn from_vector6d(vector: Vector6D) -> Self {
        Self(vector.0)
    }

    pub fn rotation(&self) -> Vector3D {
        Vector3D(self.0.fixed_rows::<3>(3).into())
    }

    pub fn translation(&self) -> Vector3D {
        Vector3D(self.0.fixed_rows::<3>(0).into())
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

    /// Computes the cross product of two spatial force vectors.
    ///
    /// # Arguments
    /// * `other` - The other spatial force vector to compute the cross product with.
    ///
    /// # Returns
    /// A new `SpatialForce` representing the cross product.
    #[must_use]
    pub fn cross(&self, other: &SpatialForce) -> SpatialForce {
        let angular = self.rotation();
        let linear = self.translation();

        let cross_matrix = SpatialForce::cross_matrix(angular, linear);

        SpatialForce(cross_matrix * other.0)
    }

    /// Computes the inner product of two spatial forces.
    #[must_use]
    pub fn inner(&self, other: &SpatialForce) -> f64 {
        self.0.dot(&other.0)
    }
}

impl Add for SpatialForce {
    type Output = SpatialForce;

    fn add(self, rhs: Self) -> Self::Output {
        SpatialForce(self.0 + rhs.0)
    }
}

impl Add<&SpatialForce> for SpatialForce {
    type Output = SpatialForce;

    fn add(self, rhs: &Self) -> Self::Output {
        SpatialForce(self.0 + rhs.0)
    }
}

impl AddAssign for SpatialForce {
    fn add_assign(&mut self, rhs: Self) {
        self.0 += rhs.0;
    }
}

impl AddAssign<&SpatialForce> for SpatialForce {
    fn add_assign(&mut self, rhs: &Self) {
        self.0 += rhs.0;
    }
}

impl Sub for SpatialForce {
    type Output = SpatialForce;

    fn sub(self, rhs: Self) -> Self::Output {
        SpatialForce(self.0 - rhs.0)
    }
}

impl Sub<&SpatialForce> for SpatialForce {
    type Output = SpatialForce;

    fn sub(self, rhs: &Self) -> Self::Output {
        SpatialForce(self.0 - rhs.0)
    }
}

impl SubAssign for SpatialForce {
    fn sub_assign(&mut self, rhs: Self) {
        self.0 -= rhs.0;
    }
}

impl SubAssign<&SpatialForce> for SpatialForce {
    fn sub_assign(&mut self, rhs: &Self) {
        self.0 -= rhs.0;
    }
}

impl ActSE3 for SpatialForce {
    fn act(&self, transform: &SE3) -> Self {
        let rotation = transform.rotation();
        let translation = transform.translation();

        let force = rotation * &self.translation();
        let torque = rotation * &self.rotation() + translation.cross(&force);

        SpatialForce::from_parts(force, torque)
    }

    fn act_inv(&self, _transform: &SE3) -> Self {
        unimplemented!()
    }
}
