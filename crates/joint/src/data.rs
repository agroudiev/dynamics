use nalgebra::IsometryMatrix3;

use crate::joint::Joint;

pub type JointDataWrapper = Box<dyn JointData + Send + Sync>;

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum JointError {
    MissingAttributeError(String),
}

pub trait JointData {
    /// Returns the placement of the joint in the world frame.
    fn get_joint_placement(&self) -> IsometryMatrix3<f64>;

    /// Updates the joint data with the given model and angle.
    fn update(&mut self, joint_model: &dyn Joint, angle: f64) -> Result<(), JointError>;
}
