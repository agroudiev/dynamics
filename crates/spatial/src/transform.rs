use nalgebra::Matrix6;

use crate::motion::SpatialRotation;

#[derive(Clone, Copy, Debug, PartialEq)]
/// Spatial transformation, represented as a 6x6 matrix.
///
/// A spatial transformation consists of a rotation and a translation, and is used to transform spatial motion and force vectors.
pub struct SpatialTransform(Matrix6<f64>);

impl SpatialTransform {
    pub fn identity() -> Self {
        SpatialTransform(Matrix6::identity())
    }

    pub fn from_rotation(rotation: SpatialRotation) -> Self {
        let mut mat = Matrix6::zeros();
        mat.view_mut((0, 0), (3, 3)).copy_from(rotation.0.matrix());
        mat.view_mut((3, 3), (3, 3)).copy_from(rotation.0.matrix());
        SpatialTransform(mat)
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
            0.0, -1.0, 0.0, 0.0, 0.0, 0.0,
            1.0,  0.0, 0.0, 0.0, 0.0, 0.0,
            0.0,  0.0, 1.0, 0.0, 0.0, 0.0,
            0.0,  0.0, 0.0, 0.0, -1.0, 0.0,
            0.0,  0.0, 0.0, 1.0,  0.0, 0.0,
            0.0,  0.0, 0.0, 0.0,  0.0, 1.0,
        );

        assert_relative_eq!(transform.0, expected);
    }
}
