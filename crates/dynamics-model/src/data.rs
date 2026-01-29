//! `Data` structure containing the mutable properties of the robot.

use dynamics_joint::joint_data::JointDataWrapper;
use dynamics_spatial::{motion::SpatialMotion, se3::SE3};

use crate::geometry_model::GeometryModel;

/// Structure containing the mutable properties of the robot.
#[derive(Default)]
pub struct Data {
    /// The data of the joints
    pub joint_data: Vec<JointDataWrapper>,
    /// The placements of the joints in the world frame (oMi)
    pub joint_placements: Vec<SE3>,
    /// Placements of the frames in the world frame (oMf)
    pub frame_placements: Vec<SE3>,
    /// Velocities of the joints in the world frame (v)
    pub joint_velocities: Vec<SpatialMotion>,
    /// Accelerations of the joints in the world frame (a)
    pub joint_accelerations: Vec<SpatialMotion>,
}

impl Data {
    /// Creates a new `Data` object.
    ///
    /// # Arguments
    ///
    /// * `joints_data` - A `HashMap` of joint indices to their data.
    /// * `joints_placements` - A `HashMap` of joint indices to their placements.
    ///
    /// # Returns
    /// A new `Data` object.
    #[must_use]
    pub fn new(
        joint_data: Vec<JointDataWrapper>,
        joint_placements: Vec<SE3>,
        frame_placements: Vec<SE3>,
        joint_velocities: Vec<SpatialMotion>,
        joint_accelerations: Vec<SpatialMotion>,
    ) -> Self {
        Self {
            joint_data,
            joint_placements,
            frame_placements,
            joint_velocities,
            joint_accelerations,
        }
    }
}

/// Structure containing the mutable geometric data of the models.
#[derive(Default)]
pub struct GeometryData {
    /// The placements of the objects in the world frame
    pub object_placements: Vec<SE3>,
}

impl GeometryData {
    /// Returns the placement of the object of given index in the world frame.
    ///
    /// # Arguments
    ///
    /// * `object_index` - The index of the object.
    ///
    /// # Returns
    /// An `Option` containing the object placement if it exists, otherwise `None`.
    #[must_use]
    pub fn get_object_placement(&self, object_index: usize) -> Option<&SE3> {
        self.object_placements.get(object_index)
    }

    /// Updates the geometry data with the given model and geometry model.
    ///
    /// # Arguments
    ///
    /// * `model` - The model containing the updated joint placements.
    /// * `data` - The data containing the joint placements.
    /// * `geom_model` - The geometry model containing the object placements.
    ///
    /// # Note
    /// As this function uses the joint placements from the model data (`data`), it should be called after the model data is updated.
    pub fn update_geometry_data(&mut self, data: &Data, geom_model: &GeometryModel) {
        self.object_placements.clear();

        for object in &geom_model.objects {
            let parent_joint_id = object.parent_joint;
            let parent_joint_placement = data.joint_placements[parent_joint_id];
            let object_placement = parent_joint_placement * object.placement;
            self.object_placements.push(object_placement);
        }
    }
}
