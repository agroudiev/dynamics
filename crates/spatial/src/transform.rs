use nalgebra::Matrix6;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SpatialTransform(Matrix6<f64>);

impl SpatialTransform {
    pub fn identity() -> Self {
        SpatialTransform(Matrix6::identity())
    }

    pub fn from_rotation(rotation: nalgebra::Rotation3<f64>) -> Self {
        let mut mat = Matrix6::zeros();
        mat.view_mut((0, 0), (3, 3)).copy_from(rotation.matrix());
        SpatialTransform(mat)
    }
}
