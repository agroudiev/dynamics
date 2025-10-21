use std::ops::Mul;

use nalgebra::Matrix6;

use crate::motion::{SpatialMotion, SpatialRotation};

#[derive(Clone, Copy, Debug, PartialEq)]
/// Spatial transformation, represented as a 6x6 matrix.
///
/// A spatial transformation consists of a rotation and a translation, and is used to transform spatial motion and force vectors.
pub struct SpatialTransform(Matrix6<f64>);

impl SpatialTransform {
    pub fn identity() -> Self {
        SpatialTransform(Matrix6::identity())
    }

    /// Creates a spatial transformation from a rotation.
    ///
    /// The resulting spatial transformation has the rotation in both the top-left and bottom-right 3x3 blocks, and zeros elsewhere:
    /// ```
    /// [ R  0 ]
    /// [ 0  R ]
    /// ```
    pub fn from_rotation(rotation: SpatialRotation) -> Self {
        let mut mat = Matrix6::zeros();
        mat.view_mut((0, 0), (3, 3)).copy_from(rotation.0.matrix());
        mat.view_mut((3, 3), (3, 3)).copy_from(rotation.0.matrix());
        SpatialTransform(mat)
    }

    /// Converts an SE(3) transformation to a spatial transformation.
    /// This is obtained using the adjoint representation of SE(3).
    ///
    /// If `R` is the rotation matrix and `t` is the translation vector of the SE(3) transformation, then the spatial transformation is given by:
    /// ```
    /// [ R          0          ]
    /// [ skew(t)*R  R          ]
    /// ```
    pub fn from_se3(se3: &crate::se3::SE3) -> Self {
        let rotation = se3.rotation().0;
        let translation = se3.translation().0;

        let mut mat = Matrix6::zeros();

        // top-left block: rotation
        mat.view_mut((0, 0), (3, 3)).copy_from(rotation.matrix());

        // top-right block: skew(translation) * rotation
        let skew_translation = nalgebra::Matrix3::new(
            0.0,
            -translation.z,
            translation.y,
            translation.z,
            0.0,
            -translation.x,
            -translation.y,
            translation.x,
            0.0,
        );
        let top_right = skew_translation * rotation.matrix();
        mat.view_mut((0, 3), (3, 3)).copy_from(&top_right);

        // bottom-left block: zero
        // bottom-right block: rotation
        mat.view_mut((3, 3), (3, 3)).copy_from(rotation.matrix());

        SpatialTransform(mat)
    }
}

impl Mul<&SpatialMotion> for &SpatialTransform {
    type Output = SpatialMotion;

    fn mul(self, rhs: &SpatialMotion) -> Self::Output {
        SpatialMotion(self.0 * rhs.0)
    }
}

impl Mul<SpatialMotion> for SpatialTransform {
    type Output = SpatialMotion;

    fn mul(self, rhs: SpatialMotion) -> Self::Output {
        SpatialMotion(self.0 * rhs.0)
    }
}

impl Mul<&SpatialMotion> for SpatialTransform {
    type Output = SpatialMotion;

    fn mul(self, rhs: &SpatialMotion) -> Self::Output {
        SpatialMotion(self.0 * rhs.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::vector3d::Vector3D;
    use approx::assert_relative_eq;

    #[test]
    fn test_identity() {
        let identity = SpatialTransform::identity();
        assert_eq!(identity.0, Matrix6::identity());
    }

    #[test]
    fn test_from_rotation_identity() {
        let rotation = SpatialRotation::identity();
        let transform = SpatialTransform::from_rotation(rotation);

        let expected = Matrix6::identity();
        assert_eq!(transform.0, expected);
    }

    #[test]
    fn test_from_rotation_pi_2() {
        let z = Vector3D::new(0.0, 0.0, 1.0);
        let rotation = SpatialRotation::from_axis_angle(&z, std::f64::consts::PI / 2.0);

        let transform = SpatialTransform::from_rotation(rotation);

        let expected = Matrix6::new(
            0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 1.0,
        );

        assert_relative_eq!(transform.0, expected);
    }
}
