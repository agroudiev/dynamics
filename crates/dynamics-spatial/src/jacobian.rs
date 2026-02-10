//! Defines a **Jacobian** structure and related operations.

use nalgebra::DMatrix;

use crate::force::SpatialForce;

/// Jacobian matrix, relating joint velocities to end-effector velocities.
///
/// The Jacobian is a matrix of size $6 \times n_v$ where $n_v$ is the number of joints in the robot model.
/// Each column corresponds to the basis of the spatial velocity
/// of the joint in the origin of the inertial frame.
#[derive(Debug, Clone, PartialEq)]
pub struct Jacobian(pub(crate) DMatrix<f64>);

impl Jacobian {
    /// Creates a zero Jacobian matrix with the given number of columns.
    ///
    /// # Arguments
    //// * `cols` - The number of columns of the Jacobian matrix, which corresponds to the number of joints in the robot model.
    ///
    /// # Returns
    /// * A zero Jacobian matrix of size $6 \times cols$.
    pub fn zero(cols: usize) -> Self {
        Self(DMatrix::zeros(6, cols))
    }

    /// Multiplies the transposed of a specified column of the Jacobian matrix with a spatial force.
    ///
    /// This operation is used to compute the contribution of a spatial force to the joint torques.
    ///
    /// # Arguments
    /// * `v_offset` - The index of the column of the Jacobian matrix to be multiplied. This corresponds to the index of the joint in the robot model.
    /// * `force` - The spatial force to be multiplied with the transposed of the specified column of the Jacobian matrix.
    ///
    /// # Returns
    /// * The result of the multiplication, which is a scalar value representing the contribution of the spatial force to the joint torque.
    pub fn column_mul(&self, v_offset: usize, force: &SpatialForce) -> f64 {
        let col = self.0.column(v_offset);
        col.dot(&force.0)
    }

    pub fn update_column(&mut self, v_offset: usize, column_data: &[f64; 6]) {
        self.0
            .fixed_columns_mut::<1>(v_offset)
            .copy_from(&DMatrix::from_column_slice(6, 1, column_data))
    }
}
