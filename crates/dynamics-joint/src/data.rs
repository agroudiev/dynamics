//! Structure containing the mutable properties of a joint.

use crate::joint::{JointWrapper, PyJointWrapper};
use dynamics_spatial::{
    configuration::{Configuration, PyConfiguration},
    se3::{PySE3, SE3},
};
use pyo3::{exceptions::PyValueError, prelude::*};

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
    fn get_joint_placement(&self) -> SE3;

    /// Updates the joint data with the given model and angle.
    fn update(
        &mut self,
        joint_model: &JointWrapper,
        q_joint: &Configuration,
    ) -> Result<(), JointError>;
}

/// A Python wrapper for the `JointDataWrapper` type.
#[pyclass(name = "JointData")]
pub struct PyJointDataWrapper {
    pub inner: JointDataWrapper,
}

#[pymethods]
impl PyJointDataWrapper {
    #[getter]
    pub fn joint_placement(&self) -> PySE3 {
        PySE3 {
            inner: self.inner.get_joint_placement(),
        }
    }

    #[pyo3(text_signature = "(joint_model, q_joint)")]
    pub fn update(
        &mut self,
        joint_model: &PyJointWrapper,
        q_joint: &PyConfiguration,
    ) -> PyResult<()> {
        match self
            .inner
            .update(&joint_model.inner, q_joint.to_configuration())
        {
            Ok(_) => Ok(()),
            Err(e) => Err(PyValueError::new_err(format!(
                "Failed to update joint data: {:?}",
                e
            ))),
        }
    }
}
