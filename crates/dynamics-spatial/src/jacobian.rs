//! Defines a **Jacobian** structure and related operations.

use std::ops::Mul;

use nalgebra::{DMatrix, Matrix6x1};

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

    /// Updates the specified column of the Jacobian matrix with the given data.
    ///
    /// # Arguments
    /// * `v_offset` - The index of the column to update, which corresponds to the joint index in the robot model.
    /// * `column_data` - The data to update the column with, which should be a 6-dimensional vector representing the spatial velocity basis of the joint.
    pub fn update_column(&mut self, v_offset: usize, column_data: &[f64; 6]) {
        self.0
            .fixed_columns_mut::<1>(v_offset)
            .copy_from(&DMatrix::from_column_slice(6, 1, column_data))
    }

    /// Returns a specified column of the Jacobian matrix as a `JacobianColumn` structure.
    ///
    /// # Arguments
    /// * `v_offset` - The index of the column to retrieve, which corresponds to the joint index in the robot model.
    /// # Returns
    /// * A `JacobianColumn` structure containing the specified column of the Jacobian matrix, which represents the spatial velocity basis of the joint.
    pub fn column(&self, v_offset: usize) -> JacobianColumn {
        JacobianColumn(Matrix6x1::from_column_slice(
            self.0.column(v_offset).as_slice(),
        ))
    }
}

/// A column of the Jacobian matrix, representing the spatial velocity basis of a joint.
pub struct JacobianColumn(pub Matrix6x1<f64>);

impl Mul<&SpatialForce> for &JacobianColumn {
    type Output = f64;

    fn mul(self, rhs: &SpatialForce) -> Self::Output {
        self.0.dot(&rhs.0)
    }
}
