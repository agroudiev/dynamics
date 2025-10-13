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
        SpatialTransform(mat)
    }
}
