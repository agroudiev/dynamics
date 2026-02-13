//! Defines the **special Euclidean group** SE(3) and related operations.
//!
//! This module defines the SE(3) transformation, which combines rotation and translation in 3D space.
//! An SE(3) transformation consists of a rotation matrix and a translation vector.
//! Operations such as composition, inversion, and action on points are provided.

use std::fmt::Display;

use crate::{motion::SpatialRotation, vector3d::Vector3D};
use nalgebra::{IsometryMatrix3, Matrix3, Matrix6, Translation3};

/// SE(3) transformation represented as an isometry matrix.
///
/// An SE(3) transformation combines a rotation and a translation in 3D space.
/// It combines a rotation matrix $R\in \text{SO}(3)$ and a translation vector $t \in \mathbb{R}^3$
#[derive(Clone, Debug, Copy, PartialEq, Default)]
pub struct SE3(pub(crate) IsometryMatrix3<f64>);

impl SE3 {
    /// Creates a new SE(3) transformation from a rotation (given as axis-angle) and a translation.
    #[must_use]
    pub fn new(translation: Vector3D, axis_angle: Vector3D) -> Self {
        let rotation = SpatialRotation::from_axis_angle(&axis_angle, axis_angle.norm());
        SE3::from_parts(
            Vector3D::new(translation.0.x, translation.0.y, translation.0.z),
            rotation,
        )
    }

    /// Creates a new SE(3) transformation from a rotation and a translation.
    #[must_use]
    pub fn from_parts(translation: Vector3D, rotation: SpatialRotation) -> Self {
        SE3(IsometryMatrix3::from_parts(
            Translation3::from(translation.0),
            rotation.0,
        ))
    }

    /// Creates a new identity SE(3) transformation, with $R = I_3$ and $t = 0_3$.
    #[must_use]
    pub fn identity() -> Self {
        SE3(IsometryMatrix3::identity())
    }

    /// Returns the inverse of the SE(3) transformation.
    #[must_use]
    pub fn inverse(&self) -> Self {
        SE3(self.0.inverse())
    }

    /// Returns the translation component of the SE(3) transformation.
    #[must_use]
    pub fn translation(&self) -> Vector3D {
        Vector3D(self.0.translation.vector)
    }

    /// Returns the rotation component of the SE(3) transformation.
    #[must_use]
    pub fn rotation(&self) -> SpatialRotation {
        SpatialRotation(self.0.rotation)
    }

    /// Computes the action matrix of the SE(3) transformation.
    ///
    /// Mathematically, the action matrix is:
    /// $$\begin{bmatrix}R & [t]_X R \\\\ 0 & R\end{bmatrix}$$
    /// where $R$ is the rotation matrix and $[t]_X$ is the skew-symmetric matrix of the translation vector $t$.
    pub fn action_matrix(&self) -> Matrix6<f64> {
        // FIXME: output a custom type

        let r = self.rotation();
        let r = r.0.matrix();
        let t = self.translation().0;
        let mut action_matrix = Matrix6::zeros();
        action_matrix
            .fixed_view_mut::<3, 3>(0, 0)
            .copy_from(&r.transpose());
        action_matrix
            .fixed_view_mut::<3, 3>(3, 3)
            .copy_from(&r.transpose());

        // Create skew-symmetric matrix [t]_x from translation vector
        let skew_t = Matrix3::new(0.0, -t[2], t[1], t[2], 0.0, -t[0], -t[1], t[0], 0.0);

        action_matrix
            .fixed_view_mut::<3, 3>(3, 0)
            .copy_from(&(skew_t * r).transpose());
        action_matrix
    }

    /// Computes the dual action matrix of the SE(3) transformation.
    ///
    /// Mathematically, the action matrix is:
    /// $$\begin{bmatrix}R & 0 \\\\ [t]_X R & R\end{bmatrix}$$
    /// where $R$ is the rotation matrix and $[t]_X$ is the skew-symmetric matrix of the translation vector $t$.
    pub fn dual_matrix(&self) -> Matrix6<f64> {
        // FIXME: output a custom type

        let r = self.rotation();
        let r = r.0.matrix();
        let t = self.translation().0;
        let mut action_matrix = Matrix6::zeros();
        action_matrix
            .fixed_view_mut::<3, 3>(0, 0)
            .copy_from(&r.transpose());
        action_matrix
            .fixed_view_mut::<3, 3>(3, 3)
            .copy_from(&r.transpose());

        // Create skew-symmetric matrix [t]_x from translation vector
        let skew_t = Matrix3::new(0.0, -t[2], t[1], t[2], 0.0, -t[0], -t[1], t[0], 0.0);

        action_matrix
            .fixed_view_mut::<3, 3>(0, 3)
            .copy_from(&(skew_t * r).transpose());
        action_matrix
    }

    /// Computes the inverse of the action matrix of the SE(3) transformation.
    pub fn inv_matrix(&self) -> Matrix6<f64> {
        // FIXME: output a custom type

        let r = self.rotation().0;
        let r_inv = r.matrix();
        let t = self.translation().0;

        let mut inv_matrix = nalgebra::Matrix6::zeros();
        inv_matrix.fixed_view_mut::<3, 3>(0, 0).copy_from(r_inv);
        inv_matrix.fixed_view_mut::<3, 3>(3, 3).copy_from(r_inv);

        // Create skew-symmetric matrix [t]_x from translation vector
        let skew_t = Matrix3::new(0.0, -t[2], t[1], t[2], 0.0, -t[0], -t[1], t[0], 0.0);

        inv_matrix
            .fixed_view_mut::<3, 3>(3, 0)
            .copy_from(&(-skew_t.transpose() * r_inv));
        inv_matrix
    }
}

impl std::ops::Mul for SE3 {
    type Output = SE3;

    fn mul(self, rhs: Self) -> Self::Output {
        SE3(self.0 * rhs.0)
    }
}

impl std::ops::Mul<&SE3> for &SE3 {
    type Output = SE3;

    fn mul(self, rhs: &SE3) -> Self::Output {
        SE3(self.0 * rhs.0)
    }
}

impl std::ops::Mul<SE3> for &SE3 {
    type Output = SE3;

    fn mul(self, rhs: SE3) -> Self::Output {
        SE3(self.0 * rhs.0)
    }
}

impl std::ops::Mul<&SE3> for SE3 {
    type Output = SE3;

    fn mul(self, rhs: &SE3) -> Self::Output {
        SE3(self.0 * rhs.0)
    }
}

impl Display for SE3 {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let r = self.0.rotation.matrix();

        writeln!(f, "SE3: R=┌                            ┐  t=┌          ┐")?;
        for i in 0..3 {
            writeln!(
                f,
                "       │ {:>+8.5} {:>+8.5} {:>+8.5} │    │ {:>+8.5} │",
                r[(i, 0)],
                r[(i, 1)],
                r[(i, 2)],
                self.0.translation.vector[i]
            )?;
        }
        writeln!(f, "       └                            ┘    └          ┘")?;
        Ok(())
    }
}

/// Trait for objects on which SE(3) transformations can act.
pub trait ActSE3 {
    /// Applies the SE(3) transformation to the object.
    fn act(&self, se3: &SE3) -> Self;

    /// Applies the inverse SE(3) transformation to the object.
    fn act_inv(&self, se3: &SE3) -> Self;
}

impl SE3 {
    /// Applies the SE(3) transformation to an object implementing the [`ActSE3`] trait.
    pub fn act<T: ActSE3>(&self, obj: &T) -> T {
        obj.act(self)
    }

    /// Applies the inverse SE(3) transformation to an object implementing the [`ActSE3`] trait.
    pub fn act_inv<T: ActSE3>(&self, obj: &T) -> T {
        obj.act_inv(self)
    }
}
