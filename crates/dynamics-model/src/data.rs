//! `Data` structure containing the mutable properties of the robot.

use dynamics_joint::joint_data::JointDataWrapper;
use dynamics_spatial::{
    configuration::Configuration, force::SpatialForce, jacobian::Jacobian, motion::SpatialMotion,
    se3::SE3, vector3d::Vector3D,
};

use crate::{geometry_model::GeometryModel, model::Model};

/// Structure containing the mutable properties of the robot.
pub struct Data {
    /// The data of the joints
    pub joint_data: Vec<JointDataWrapper>,
    /// The placements of the joints in the world frame (oMi)
    pub joint_placements: Vec<SE3>,
    /// Placements of the frames in the world frame (oMf)
    pub frame_placements: Vec<SE3>,
    /// The local joint placements in the parent frame (liMi)
    pub local_joint_placements: Vec<SE3>,
    /// Velocities of the joints in the local frame (v)
    pub joint_velocities: Vec<SpatialMotion>,
    /// Velocities of the joints in the world frame (ov)
    pub world_joint_velocities: Vec<SpatialMotion>,
    /// Accelerations of the joints in the local frame (a)
    pub joint_accelerations: Vec<SpatialMotion>,
    /// Accelerations of the joints due to the gravity field (a_gf)
    pub joint_accelerations_gravity_field: Vec<SpatialMotion>,
    /// Accelerations of the joints in the world frame including the gravity field (oa_gf)
    pub world_accelerations_gravity_field: Vec<SpatialMotion>,
    /// The spatial momenta of the joint in the local frame (h), inertia times velocity
    pub joint_momenta: Vec<SpatialForce>,
    /// The spatial forces of the joint in the local frame (f), inertia times acceleration plus the Coriolis term
    pub joint_forces: Vec<SpatialForce>,
    /// The spatial forces of the joint in the world frame (of), inertia times acceleration plus the Coriolis term
    pub world_joint_forces: Vec<SpatialForce>,
    /// The configuration of torques/forces applied to the joints (tau)
    pub tau: Configuration,
    /// The joint accelerations of the joints computed by the forward dynamics (ddq)
    pub ddq: Configuration,
    /// The Jacobian matrix of the joint placements (J)
    pub jacobian: Jacobian,
}

impl Data {
    /// Creates a new `Data` object from the given joint datas and model.
    ///
    /// # Arguments
    ///
    /// * `joint_data` - A vector of joint data wrappers.
    /// * `model` - The robot model.
    ///
    /// # Returns
    /// A new `Data` object.
    #[must_use]
    pub fn from_joints_data(joint_data: Vec<JointDataWrapper>, model: &Model) -> Self {
        let njoints = joint_data.len();
        let mut joint_accelerations_gravity_field = vec![SpatialMotion::zero(); njoints];

        // set the base acceleration to compensate gravity
        joint_accelerations_gravity_field[0] =
            SpatialMotion::from_parts(-model.gravity, Vector3D::zeros());

        Data {
            joint_data,
            joint_placements: vec![SE3::identity(); njoints],
            frame_placements: vec![SE3::identity(); model.nframes()],
            local_joint_placements: vec![SE3::identity(); njoints],
            joint_velocities: vec![SpatialMotion::zero(); njoints],
            world_joint_velocities: vec![SpatialMotion::zero(); njoints],
            joint_accelerations: vec![SpatialMotion::zero(); njoints],
            joint_accelerations_gravity_field,
            world_accelerations_gravity_field: vec![SpatialMotion::zero(); njoints],
            joint_momenta: vec![SpatialForce::zero(); njoints],
            joint_forces: vec![SpatialForce::zero(); njoints],
            world_joint_forces: vec![SpatialForce::zero(); njoints],
            tau: Configuration::zeros(model.nv),
            ddq: Configuration::zeros(model.nv),
            jacobian: Jacobian::zero(model.nv),
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
