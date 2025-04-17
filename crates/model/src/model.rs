//! The standard `Model`.

use joint::{
    joint::{Joint, JointWrapper, PyJointWrapper},
    revolute::PyJointModelRevolute,
};
use nalgebra::IsometryMatrix3;
use pyo3::{exceptions::PyValueError, prelude::*, types::PyTuple};
use spatial::se3::PySE3;
use std::collections::HashMap;

/// A `Model` is a data structure that contains the information about the robot model,
/// including the joints models, placements, the link inertias, and the frames.
pub struct Model {
    /// The name of the model.
    pub name: String,
    /// The names of the joints.
    pub joint_names: HashMap<usize, String>,
    /// The placements of the joints.
    pub joint_placements: HashMap<usize, IsometryMatrix3<f64>>,
    /// The joint models.
    pub joint_models: HashMap<usize, JointWrapper>,
    /// The order of the joints.
    joint_order: Vec<usize>,
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
        Self {
            name,
            joint_names: HashMap::new(),
            joint_placements: HashMap::new(),
            joint_models: HashMap::new(),
            joint_order: Vec::new(),
            nq: 0,
            nv: 0,
        }
    }

    /// Creates a new empty `Model`.
    pub fn new_empty() -> Self {
        Self {
            name: String::new(),
            joint_names: HashMap::new(),
            joint_placements: HashMap::new(),
            joint_models: HashMap::new(),
            joint_order: Vec::new(),
            nq: 0,
            nv: 0,
        }
    }

    /// Adds a joint to the model.
    ///
    /// # Arguments
    ///
    /// * `parent_id` - The identifier of the parent joint.
    /// * `joint_model` - The joint model to add.
    /// * `placement` - The placement of the joint in the parent frame.
    /// * `name` - The name of the joint.
    pub fn add_joint(
        &mut self,
        _parent_id: usize,
        joint_model: JointWrapper,
        placement: IsometryMatrix3<f64>,
        name: String,
    ) -> usize {
        let id = self.joint_names.len();
        self.joint_names.insert(id, name);
        self.joint_placements.insert(id, placement);
        self.nq += joint_model.nq();
        self.nv += joint_model.nv();
        self.joint_models.insert(id, joint_model);
        self.joint_order.push(id);

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
