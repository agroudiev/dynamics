//! Structure containing the mutable properties of a joint.

use crate::joint::JointWrapper;
use nalgebra::{DVector, IsometryMatrix3};

/// Dynamic type for a joint.
pub type JointDataWrapper = Box<dyn JointData + Send + Sync>;

/// Error type for joint data operations.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum JointError {
    MissingAttributeError(String),
}

/// Trait for joint data, providing methods to access and update joint properties.
pub trait JointData {
    /// Returns the placement of the joint in the world frame.
    fn get_joint_placement(&self) -> IsometryMatrix3<f64>;

    /// Updates the joint data with the given model and angle.
    fn update(
        &mut self,
        joint_model: &JointWrapper,
        q_joint: &DVector<f64>,
    ) -> Result<(), JointError>;
}
