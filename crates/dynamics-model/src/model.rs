//! [`Model`] structure containing the robot model and its immutable properties.

use crate::data::Data;
use crate::frame::{Frame, FrameType};
use dynamics_inertia::inertia::Inertia;
use dynamics_joint::fixed::JointModelFixed;
use dynamics_joint::joint::{JointModel, JointWrapper};
use dynamics_spatial::configuration::Configuration;
use dynamics_spatial::motion::SpatialMotion;
use dynamics_spatial::se3::SE3;
use dynamics_spatial::vector3d::Vector3D;

use std::fmt::{Debug, Display};
use std::sync::LazyLock;

/// Identifier of the world joint.
pub const WORLD_ID: usize = 0;
/// Standard gravity vector (0, 0, -9.81).
pub static STANDARD_GRAVITY: LazyLock<Vector3D> = LazyLock::new(|| Vector3D::new(0.0, 0.0, -9.81));

/// Data structure that contains the immutable properties of the robot model.
/// It contains information about the joints, frames, and their local placements.
pub struct Model {
    /// Name of the model.
    pub name: String,
    /// Names of the joints.
    pub joint_names: Vec<String>,
    /// Parent joint of each joint.
    pub joint_parents: Vec<usize>,
    /// Placements of the joints relative to their parent joints.
    pub joint_placements: Vec<SE3>,
    /// Joint models.
    pub joint_models: Vec<JointWrapper>,
    /// Number of position variables.
    pub nq: usize,
    /// Number of velocity variables.
    pub nv: usize,
    /// Inertias of the bodies at each joint.
    pub inertias: Vec<Inertia>,
    /// Operational frames at each joint
    pub frames: Vec<Frame>,
    /// The spatial gravity of the model.
    pub gravity: Vector3D, // TODO: replace this by a SpatialMotion
}

impl Model {
    /// Creates a new [`Model`] with given name.
    ///
    /// Same as [`Model::new_empty()`].
    ///
    /// # Arguments
    ///
    /// * `name` - The name of the model.
    #[must_use]
    pub fn new(name: String) -> Self {
        let mut model = Self::new_empty();
        model.name = name;
        model
    }

    /// Creates a new empty [`Model`].
    ///
    /// # Returns
    ///
    /// A new empty [`Model`].
    #[must_use]
    pub fn new_empty() -> Self {
        Self {
            name: String::new(),
            joint_names: vec!["__WORLD__".to_string()],
            joint_parents: vec![WORLD_ID],
            joint_placements: vec![SE3::identity()],
            joint_models: vec![JointWrapper::fixed(JointModelFixed::default())],
            nq: 0,
            nv: 0,
            inertias: vec![Inertia::zeros()],
            frames: vec![Frame::new(
                "__WORLD_FRAME__".to_string(),
                WORLD_ID,
                WORLD_ID,
                SE3::identity(),
                FrameType::Fixed,
                Inertia::zeros(),
            )],
            gravity: *STANDARD_GRAVITY,
        }
    }

    /// Adds a joint to the model.
    ///
    /// # Arguments
    ///
    /// * `parent_id` - The identifier of the parent joint. Use 0 for the root joint.
    /// * `joint_model` - The joint model to add.
    /// * `placement` - The placement of the joint in the parent frame.
    /// * `name` - The name of the joint.
    pub fn add_joint(
        &mut self,
        parent_id: usize,
        joint_model: JointWrapper,
        placement: SE3,
        name: String,
    ) -> Result<usize, ModelError> {
        if parent_id >= self.joint_names.len() {
            return Err(ModelError::ParentJointDoesNotExist(parent_id));
        }
        for (id, other_name) in self.joint_names.iter().enumerate() {
            if other_name == &name {
                return Err(ModelError::JointNameAlreadyUsed(name, id));
            }
        }

        let id = self.joint_names.len();
        self.joint_names.push(name);
        self.joint_placements.push(placement);
        self.nq += joint_model.nq();
        self.nv += joint_model.nv();
        self.joint_models.push(joint_model);
        self.inertias.push(Inertia::zeros());

        // add the joint to the parent
        self.joint_parents.push(parent_id);
        Ok(id)
    }

    /// Adds a frame (fixed joint) to the model.
    ///
    /// # Arguments
    ///
    /// * `placement` - The placement of the frame in the parent frame.
    /// * `name` - The name of the frame.
    ///
    /// # Returns
    ///
    /// The identifier of the frame.
    pub fn add_frame(&mut self, frame: Frame, append_inertia: bool) -> Result<usize, ModelError> {
        // check if the parent exists
        if frame.parent_joint >= self.joint_names.len() {
            return Err(ModelError::ParentJointDoesNotExist(frame.parent_joint));
        }

        // check if a frame with the same name and type exists
        for (id, other_frame) in self.frames.iter().enumerate() {
            if other_frame.name == frame.name && other_frame.frame_type == frame.frame_type {
                return Ok(id);
            }
        }

        // otherwise, add the frame
        let id = self.frames.len();
        self.frames.push(frame);
        let frame = &self.frames[id];

        if append_inertia {
            self.inertias[frame.parent_joint] += frame.placement.act(&frame.inertia);
        }

        Ok(id)
    }

    /// Creates the data associated with the model.
    ///
    /// # Returns
    ///
    /// The data associated with the model.
    #[must_use]
    pub fn create_data(&self) -> Data {
        let joints_data = self
            .joint_models
            .iter()
            .map(|joint_model| joint_model.create_joint_data())
            .collect();

        Data::new(
            joints_data,
            vec![SE3::identity(); self.njoints()],
            vec![SE3::identity(); self.nframes()],
            vec![SE3::identity(); self.njoints()],
            vec![SpatialMotion::zero(); self.njoints()],
            vec![SpatialMotion::zero(); self.njoints()],
        )
    }

    // /// Appends a body of given inertia to the joint with given id.
    // ///
    // /// # Arguments
    // ///
    // /// * `joint_id` - The identifier of the joint to append the body to.
    // /// * [`Inertia`] - The inertia of the body to append.
    // /// * `placement` - The placement of the body in the joint frame.
    // ///
    // /// # Returns
    // ///
    // /// A result indicating success or failure.
    // pub fn append_body_to_joint(
    //     &mut self,
    //     joint_id: usize,
    //     inertia: Inertia,
    //     placement: SE3,
    // ) -> Result<(), ModelError> {
    //     if !self.joint_names.contains_key(&joint_id) {
    //         return Err(ModelError::ParentJointDoesNotExist(joint_id));
    //     }

    //     self.inertias.insert(joint_id, inertia);
    //     self.body_placements.insert(joint_id, placement);

    //     Ok(())
    // }

    /// Returns the index of the joint with the given name.
    #[must_use]
    pub fn get_joint_id(&self, name: &str) -> Option<usize> {
        for (id, joint_name) in self.joint_names.iter().enumerate() {
            if joint_name == name {
                return Some(id);
            }
        }
        None
    }

    /// Returns the index of the frame with the given name.
    ///
    /// # Arguments
    /// * `name` - The name of the frame.
    /// * `frame_type` - The type of the frame.
    ///
    /// # Returns
    /// The index of the frame with the given name and type, or `None` if not found.
    #[must_use]
    pub fn get_frame_id(&self, name: &str, frame_type: Option<FrameType>) -> Option<usize> {
        for (id, frame) in self.frames.iter().enumerate() {
            if frame.name == name {
                if let Some(ft) = &frame_type
                    && &frame.frame_type != ft
                {
                    continue;
                }
                return Some(id);
            }
        }
        None
    }

    /// Returns the number of joints in the model, including the world frame.
    #[must_use]
    pub fn njoints(&self) -> usize {
        self.joint_names.len()
    }

    /// Returns the number of frames in the model, including the world frame.
    #[must_use]
    pub fn nframes(&self) -> usize {
        self.frames.len()
    }
}

impl Debug for Model {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Model")
            .field("name", &self.name)
            .field("joint_names", &self.joint_names)
            .field("joint_parents", &self.joint_parents)
            .field("joint_placements", &self.joint_placements)
            // .field("joint_models", &self.joint_models)
            .finish()
    }
}

#[derive(Debug)]
/// An error that can occur when adding a joint to the model.
pub enum ModelError {
    /// The parent joint does not exist.
    ParentJointDoesNotExist(usize),
    /// The name of the joint is already used.
    JointNameAlreadyUsed(String, usize),
}

impl Display for ModelError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ModelError::ParentJointDoesNotExist(id) => {
                write!(f, "Parent joint with id {} does not exist.", id)
            }
            ModelError::JointNameAlreadyUsed(name, id) => {
                write!(
                    f,
                    "Joint name '{}' is already used by joint with id {}.",
                    name, id
                )
            }
        }
    }
}

/// Generates a random configuration for the given model.
#[must_use]
pub fn random_configuration(model: &Model) -> Configuration {
    let mut rng = rand::rng();
    let q = model
        .joint_models
        .iter()
        .map(|joint_model| joint_model.random_configuration(&mut rng))
        .collect::<Vec<_>>();
    Configuration::concat(q.as_slice())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_create_empty_model() {
        let model = Model::new_empty();
        assert_eq!(model.name, "");
        assert_eq!(model.njoints(), 1);
        assert_eq!(model.nq, 0);
        assert_eq!(model.nv, 0);
    }

    #[test]
    fn create_data_empty_model() {
        let model = Model::new_empty();
        let data = model.create_data();
        assert_eq!(data.joint_placements.len(), model.njoints());
        assert_eq!(data.joint_placements, vec![SE3::identity()]);
    }
}
