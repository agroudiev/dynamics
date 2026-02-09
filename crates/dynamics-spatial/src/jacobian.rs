//! Defines a **Jacobian** structure and related operations.

use nalgebra::DMatrix;

/// Jacobian matrix, relating joint velocities to end-effector velocities.
///
/// The Jacobian is a matrix of size $6 \times n_v$ where $n_v$ is the number of joints in the robot model.
/// Each column corresponds to the basis of the spatial velocity
/// of the joint in the origin of the inertial frame.
#[derive(Debug, Clone, PartialEq)]
pub struct Jacobian(pub(crate) DMatrix<f64>);

impl Jacobian {
    pub fn zero(cols: usize) -> Self {
        Self(DMatrix::zeros(6, cols))
    }

    pub fn update_column(&mut self, v_offset: usize, column_data: &[f64; 6]) {
        self.0
            .fixed_columns_mut::<1>(v_offset)
            .copy_from(&DMatrix::from_column_slice(6, 1, column_data))
    }
}
