//! Structures and traits for data representation.
//! The `Model` object represents the properties of the robot model which are immutable and are not changed or computed by the algorithms.
//! The `Data` object represents the properties of the robot model which are obtained by the algorithms, such as the joint positions, velocities, accelerations.

use joint::data::JointDataWrapper;
use nalgebra::IsometryMatrix3;
use std::collections::HashMap;

#[derive(Default)]
pub struct Data {
    /// The data fo the joints
    joints_data: HashMap<usize, JointDataWrapper>,
    /// The placements of the joints in the world frame
    joints_placements: HashMap<usize, IsometryMatrix3<f64>>,
}

impl Data {
    /// Creates a new `Data` object.
    ///
    /// # Arguments
    ///
    /// * `joints_data` - A HashMap of joint indices to their data.
    /// * `joints_placements` - A HashMap of joint indices to their placements.
    ///
    /// # Returns
    /// A new `Data` object.
    pub fn new(
        joints_data: HashMap<usize, JointDataWrapper>,
        joints_placements: HashMap<usize, IsometryMatrix3<f64>>,
    ) -> Self {
        Self {
            joints_data,
            joints_placements,
        }
    }

    /// Returns the joint data for a given joint index.
    ///
    /// # Arguments
    ///
    /// * `joint_index` - The index of the joint.
    ///
    /// # Returns
    /// An `Option` containing the joint data if it exists, otherwise `None`.
    pub fn get_joint_data(&self, joint_index: usize) -> Option<&JointDataWrapper> {
        self.joints_data.get(&joint_index)
    }

    /// Returns the joint placement for a given joint index.
    ///
    /// # Arguments
    ///
    /// * `joint_index` - The index of the joint.
    ///
    /// # Returns
    /// An `Option` containing the joint placement if it exists, otherwise `None`.
    pub fn get_joint_placement(&self, joint_index: usize) -> Option<&IsometryMatrix3<f64>> {
        self.joints_placements.get(&joint_index)
    }
}
