#[derive(Debug, Clone, PartialEq)]
pub struct SpatialInertia(nalgebra::Matrix6<f64>);

impl SpatialInertia {
    pub fn from_diagonal(diag: &nalgebra::Vector6<f64>) -> Self {
        let mut mat = nalgebra::Matrix6::zeros();
        mat[(0, 0)] = diag[0];
        mat[(1, 1)] = diag[1];
        mat[(2, 2)] = diag[2];
        mat[(3, 3)] = diag[3];
        mat[(4, 4)] = diag[4];
        mat[(5, 5)] = diag[5];
        Self(mat)
    }
}
