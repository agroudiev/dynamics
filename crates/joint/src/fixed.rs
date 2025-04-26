//! Fixed joint, without any degree of freedom.

use crate::{
    data::{JointData, JointDataWrapper, JointError},
    joint::{Joint, JointType, JointWrapper},
};
use nalgebra::IsometryMatrix3;
use pyo3::prelude::*;

/// Model of a fixed joint.
#[derive(Clone, Debug)]
pub struct JointModelFixed {}

impl Joint for JointModelFixed {
    fn get_joint_type(&self) -> JointType {
        JointType::Fixed
    }

    fn clone_box(&self) -> JointWrapper {
        Box::new(self.clone())
    }

    fn nq(&self) -> usize {
        0
    }

    fn nv(&self) -> usize {
        0
    }

    fn neutral(&self) -> Vec<f64> {
        vec![]
    }

    fn create_joint_data(&self) -> JointDataWrapper {
        Box::new(JointDataFixed::new(self))
    }
}

/// Data structure containing the mutable properties of a fixed joint.
#[derive(Default, Debug)]
pub struct JointDataFixed {
    /// The placement of the joint in the local frame.
    pub placement: IsometryMatrix3<f64>,
}

impl JointDataFixed {
    /// Creates a new `JointDataFixed` from given joint model.
    ///
    /// # Returns
    /// A new `JointDataFixed` object.
    pub fn new(joint_model: &JointModelFixed) -> Self {
        let mut data = JointDataFixed::default();
        // safe since we just created a revolute joint model
        // and we know that a revolute joint has an axis
        data.update(joint_model, 0.0).unwrap();
        data
    }
}

impl JointData for JointDataFixed {
    fn get_joint_placement(&self) -> IsometryMatrix3<f64> {
        self.placement
    }

    fn update(&mut self, _joint_model: &dyn Joint, _q: f64) -> Result<(), JointError> {
        Ok(())
    }
}

/// A Python wrapper for the `JointModelFixed` struct.
#[pyclass(name = "JointModelFixed")]
pub struct PyJointModelFixed {
    pub inner: JointModelFixed,
}
