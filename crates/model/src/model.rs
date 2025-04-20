//! The standard `Model`.

use crate::data::Data;
use joint::{
    joint::{Joint, JointWrapper, PyJointWrapper},
    revolute::PyJointModelRevolute,
};
use nalgebra::IsometryMatrix3;
use pyo3::{exceptions::PyValueError, prelude::*, types::PyTuple};
use spatial::se3::PySE3;
use std::collections::HashMap;

pub const WORLD_FRAME_ID: usize = 0;

/// A `Model` is a data structure that contains the information about the robot model,
/// including the joints models, placements, the link inertias, and the frames.
pub struct Model {
    /// The name of the model.
    pub name: String,
    /// The names of the joints.
    pub joint_names: HashMap<usize, String>,
    /// The parent joint of each joint.
    pub joint_parents: HashMap<usize, usize>,
    /// The placements of the joints.
    pub joint_placements: HashMap<usize, IsometryMatrix3<f64>>,
    /// The joint models.
    joint_models: HashMap<usize, JointWrapper>,
    /// The order of the joints.
    joint_order: Vec<usize>,
    /// The frames of the model.
    pub frames: HashMap<usize, IsometryMatrix3<f64>>,
    /// The number of position variables.
    pub nq: usize,
    /// The number of velocity variables.
    pub nv: usize,
}

impl Model {
    /// Creates a new `Model` with given name.
    ///
    /// # Arguments
    ///
    /// * `name` - The name of the model.
    pub fn new(name: String) -> Self {
        let mut model = Self::new_empty();
        model.name = name;
        model
    }

    /// Creates a new empty `Model`.
    pub fn new_empty() -> Self {
        let mut frames = HashMap::new();
        frames.insert(WORLD_FRAME_ID, IsometryMatrix3::identity());

        let mut joint_parents = HashMap::new();
        joint_parents.insert(WORLD_FRAME_ID, WORLD_FRAME_ID);

        let mut joint_names = HashMap::new();
        joint_names.insert(WORLD_FRAME_ID, "world".to_string());

        let mut joint_placements = HashMap::new();
        joint_placements.insert(WORLD_FRAME_ID, IsometryMatrix3::identity());

        Self {
            name: String::new(),
            joint_names,
            joint_parents,
            joint_placements,
            joint_models: HashMap::new(),
            joint_order: vec![WORLD_FRAME_ID],
            frames,
            nq: 0,
            nv: 0,
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
        placement: IsometryMatrix3<f64>,
        name: String,
    ) -> usize {
        let id = self.joint_order.len();
        self.joint_names.insert(id, name);
        self.joint_placements.insert(id, placement);
        self.nq += joint_model.nq();
        self.nv += joint_model.nv();
        self.joint_models.insert(id, joint_model);
        self.joint_order.push(id);

        // add the joint to the parent
        assert!(
            self.joint_parents.contains_key(&parent_id),
            "The parent joint id {} does not exist.",
            parent_id
        );
        self.joint_parents.insert(id, parent_id);

        id
    }

    /// Returns an iterator over the joint models in the model.
    /// This iterator is ordered in such a way that if the method is called twice,
    /// and if no changes are made to the model, the order of the will remain the same.
    pub fn iter_joint_models(&self) -> impl Iterator<Item = &JointWrapper> {
        self.joint_order
            .iter()
            .map(move |id| self.joint_models.get(id).unwrap())
    }

    /// Adds a frame to the model.
    ///
    /// # Arguments
    ///
    /// * `placement` - The placement of the frame in the parent frame.
    /// * `name` - The name of the frame.
    ///
    /// # Returns
    ///
    /// The identifier of the frame.
    pub fn add_frame(&mut self, placement: IsometryMatrix3<f64>, name: String) -> usize {
        let id = self.frames.len();
        self.frames.insert(id, placement);
        self.joint_names.insert(id, name);
        id
    }

    /// Creates the data associated with the model.
    ///
    /// # Returns
    ///
    /// The data associated with the model.
    pub fn create_data(&self) -> Data {
        // create the data for each joint
        let mut joints_data = HashMap::new();
        for (id, joint_model) in self.joint_models.iter() {
            let joint_data = joint_model.create_joint_data();
            joints_data.insert(*id, joint_data);
        }

        // create the placements of the joints in the world frame
        // by traversing the joint tree
        let mut joint_placements = HashMap::new();
        joint_placements.insert(WORLD_FRAME_ID, IsometryMatrix3::identity());

        for joint_id in self.joint_order.iter() {
            let parent_id = self.joint_parents.get(joint_id).unwrap(); // we checked that the parent existed before
            // get the placement of the parent join in the world frame
            let parent_placement = joint_placements.get(parent_id).unwrap();
            // get the placement of the joint in the parent frame
            let joint_placement = self.joint_placements.get(joint_id).unwrap();
            // compute the placement of the joint in the world frame
            let joint_placement_in_world = parent_placement * joint_placement;
            joint_placements.insert(*joint_id, joint_placement_in_world);
        }

        Data::new(joints_data, joint_placements)
    }
}

/// A `Model` is a data structure that contains the information about the robot model,
/// including the joints models, placements, the link inertias, and the frames.
#[pyclass(name = "Model")]
pub struct PyModel {
    pub inner: Model,
}

#[pymethods]
impl PyModel {
    /// Creates a new empty `Model`.
    // TODO: update this function for more flexibility
    #[new]
    fn new_empty() -> Self {
        Self {
            inner: Model::new_empty(),
        }
    }

    #[getter]
    fn name(&self) -> &str {
        &self.inner.name
    }

    #[setter]
    fn set_name(&mut self, name: String) {
        self.inner.name = name;
    }

    #[getter]
    fn nq(&self) -> usize {
        self.inner.nq
    }

    #[getter]
    fn nv(&self) -> usize {
        self.inner.nv
    }

    /// Adds a joint to the model.
    ///
    /// # Arguments
    ///
    /// * `parent_id` - The identifier of the parent joint.
    /// * `joint_model` - The joint model to add.
    /// * `placement` - The placement of the joint in the parent frame.
    /// * `name` - The name of the joint.
    #[pyo3(signature = (*py_args))]
    fn add_joint(&mut self, py_args: &Bound<'_, PyTuple>) -> PyResult<usize> {
        if py_args.len() == 4 {
            let parent_id: usize = py_args.get_item(0)?.extract()?;

            let joint_model: PyJointWrapper = if let Ok(revolute) = py_args
                .get_item(1)?
                .extract::<PyRef<PyJointModelRevolute>>()
            {
                PyJointWrapper {
                    inner: revolute.inner.clone_box(),
                }
            } else {
                return Err(PyValueError::new_err(format!(
                    "add_joint() second argument must be a joint model, but got {:?}.",
                    py_args.get_item(1)?.get_type()
                )));
            };

            let placement = py_args.get_item(2)?.extract::<PyRef<PySE3>>()?;
            let name: String = py_args.get_item(3)?.extract()?;

            Ok(self.inner.add_joint(
                parent_id,
                joint_model.inner.clone_box(),
                placement.inner,
                name,
            ))
        } else {
            Err(PyValueError::new_err(
                "add_joint() takes exactly 4 arguments",
            ))
        }
    }
}
