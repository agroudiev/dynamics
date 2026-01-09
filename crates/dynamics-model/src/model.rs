//! `Model` structure containing the robot model and its immutable properties.

use crate::data::{Data, PyData};
use crate::forward_kinematics::forward_kinematics;
use inertia::inertia::Inertia;
use joint::{
    joint::{Joint, JointWrapper, PyJointWrapper},
    revolute::PyJointModelRevolute,
};
use numpy::ToPyArray;
use numpy::ndarray::Array1;
use once_cell::sync::Lazy;
use pyo3::{exceptions::PyValueError, prelude::*, types::PyTuple};
use spatial::configuration::{Configuration, PyConfiguration};
use spatial::se3::{PySE3, SE3};
use spatial::vector3d::Vector3D;
use std::fmt::Debug;

pub const WORLD_FRAME_ID: usize = 0;
pub static STANDARD_GRAVITY: Lazy<Vector3D> = Lazy::new(|| Vector3D::new(0.0, 0.0, -9.81));

/// Data structure that contains the immutable properties of the robot model.
/// It contains information about the joints, frames, and their local placements.
pub struct Model {
    /// The name of the model.
    pub name: String,
    /// The names of the joints.
    pub joint_names: Vec<String>,
    /// The parent joint of each joint.
    pub joint_parents: Vec<usize>,
    /// The placements of the joints relative to their parent joints.
    pub joint_placements: Vec<SE3>,
    /// The joint models.
    pub joint_models: Vec<JointWrapper>,
    /// The number of position variables.
    pub nq: usize,
    /// The number of velocity variables.
    pub nv: usize,
    /// The inertias of the bodies.
    pub inertias: Vec<Inertia>,
    /// The spatial gravity of the model.
    pub gravity: Vector3D, // TODO: replace this by a SpartialMotion
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
    ///
    /// # Returns
    ///
    /// A new empty `Model`.
    pub fn new_empty() -> Self {
        Self {
            name: String::new(),
            joint_names: vec!["__WORLD__".to_string()],
            joint_parents: vec![WORLD_FRAME_ID],
            joint_placements: vec![SE3::identity()],
            joint_models: Vec::new(),
            nq: 0,
            nv: 0,
            inertias: Vec::new(),
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
    pub fn add_frame(
        &mut self,
        placement: SE3,
        name: String,
        parent_id: usize,
    ) -> Result<usize, ModelError> {
        let fixed_joint_model = joint::fixed::JointModelFixed::default();
        self.add_joint(parent_id, Box::new(fixed_joint_model), placement, name)
    }

    /// Creates the data associated with the model.
    ///
    /// # Returns
    ///
    /// The data associated with the model.
    pub fn create_data(&self) -> Data {
        // create the data for each joint
        let mut joints_data = Vec::with_capacity(self.joint_models.len());
        for joint_model in self.joint_models.iter() {
            let joint_data = joint_model.create_joint_data();
            joints_data.push(joint_data);
        }

        // TODO: initialize the placements
        let mut data = Data::new(joints_data, Vec::with_capacity(self.joint_models.len()));

        forward_kinematics(self, &mut data, &Configuration::zeros(self.nq))
            .expect("Failed to compute forward kinematics");

        data
    }

    // /// Appends a body of given inertia to the joint with given id.
    // ///
    // /// # Arguments
    // ///
    // /// * `joint_id` - The identifier of the joint to append the body to.
    // /// * `inertia` - The inertia of the body to append.
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

    pub fn joint_index_by_name(&self, name: &str) -> Option<usize> {
        for (id, joint_name) in self.joint_names.iter().enumerate() {
            if joint_name == name {
                return Some(id);
            }
        }
        None
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

/// Generates a random configuration for the given model.
pub fn random_configuration(model: &Model) -> Configuration {
    let mut rng = rand::rng();
    let q = model
        .joint_models
        .iter()
        .map(|joint_model| joint_model.random_configuration(&mut rng))
        .collect::<Vec<_>>();
    Configuration::concat(q.as_slice())
}

#[pyfunction(name = "random_configuration")]
pub fn py_random_configuration(model: &mut PyModel) -> PyConfiguration {
    let q = random_configuration(&model.inner);
    PyConfiguration::new(q)
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

    #[getter]
    fn gravity(&self, py: Python) -> Py<PyAny> {
        Array1::from_shape_vec([3], self.inner.gravity.as_slice().to_vec())
            .unwrap()
            .to_pyarray(py)
            .into_any()
            .unbind()
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

            match self.inner.add_joint(
                parent_id,
                joint_model.inner.clone_box(),
                placement.inner,
                name,
            ) {
                Ok(id) => Ok(id),
                Err(model_error) => Err(PyValueError::new_err(format!("{:?}", model_error))),
            }
        } else {
            Err(PyValueError::new_err(
                "add_joint() takes exactly 4 arguments",
            ))
        }
    }

    // fn append_body_to_joint(
    //     &mut self,
    //     joint_id: usize,
    //     inertia: &PyInertia,
    //     placement: &PySE3,
    // ) -> PyResult<()> {
    //     match self
    //         .inner
    //         .append_body_to_joint(joint_id, inertia.inner.clone(), placement.inner)
    //     {
    //         Ok(_) => Ok(()),
    //         Err(model_error) => Err(PyValueError::new_err(format!("{:?}", model_error))),
    //     }
    // }

    fn create_data(&self) -> PyResult<PyData> {
        let data = self.inner.create_data();
        Ok(PyData { inner: data })
    }

    fn __repr__(slf: PyRef<'_, Self>) -> String {
        format!("{:#?}", slf.inner)
    }
}
