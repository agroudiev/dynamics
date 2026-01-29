//! Structure containing the mutable properties of a joint.

use std::fmt::Display;

use crate::joint::JointWrapper;
use dynamics_spatial::{configuration::Configuration, se3::SE3};

/// Dynamic type for a joint.
pub type JointDataWrapper = Box<dyn JointData + Send + Sync>;

/// Error type for joint data operations.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum JointError {
    MissingAttributeError(String),
}

impl Display for JointError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            JointError::MissingAttributeError(attr) => {
                write!(f, "Missing attribute: {}", attr)
            }
        }
    }
}

/// Trait for joint data, providing methods to access and update joint properties.
pub trait JointData {
    /// Returns the joint configuration vector.
    fn get_joint_q(&self) -> &Configuration;

    /// Returns the joint velocity vector.
    fn get_joint_v(&self) -> &Configuration;

    /// Returns the placement of the joint in the world frame.
    fn get_joint_placement(&self) -> SE3;

    /// Updates the joint data with the given model and angle.
    fn update(
        &mut self,
        joint_model: &JointWrapper, // TODO: remove this argument
        joint_q: &Configuration,
        joint_v: Option<&Configuration>,
    ) -> Result<(), JointError>;

    /// Clones the joint data as a boxed trait object.
    fn clone_box(&self) -> JointDataWrapper;
}
