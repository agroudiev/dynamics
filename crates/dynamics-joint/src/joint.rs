//! Trait for joints and a wrapper type for dynamic dispatch.

use crate::data::JointDataWrapper;
use dynamics_spatial::{configuration::Configuration, motion::SpatialMotion, se3::SE3};
use pyo3::prelude::*;
use rand::rngs::ThreadRng;

/// A wrapper type for the Shape trait to allow dynamic dispatch.
pub type JointWrapper = Box<dyn JointModel + Send + Sync>;

/// Joint trait for defining joints in a robotic system.
pub trait JointModel {
    /// Returns the joint type.
    fn get_joint_type(&self) -> JointType;

    /// Clones the joint and returns a boxed version of it.
    fn clone_box(&self) -> JointWrapper;

    /// Returns the number of position variables.
    fn nq(&self) -> usize;

    /// Returns the number of velocity variables.
    fn nv(&self) -> usize;

    /// Returns the neutral configuration of the joint.
    fn neutral(&self) -> Configuration;

    /// Creates the joint data.
    fn create_joint_data(&self) -> JointDataWrapper;

    /// Returns the axis of the joint, if applicable.
    fn get_axis(&self) -> Vec<SpatialMotion> {
        Vec::new()
    }

    /// Returns a random configuration for the joint.
    fn random_configuration(&self, rng: &mut ThreadRng) -> Configuration;

    /// Computes the transformation matrix of the joint given its configuration. Featherstone calls it `jcalc`.
    fn transform(&self, q: &Configuration) -> SE3;
}

/// Enum representing the type of joint.
#[pyclass]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum JointType {
    Fixed,
    Revolute,
}

/// A Python wrapper for the `JointWrapper` type.
#[pyclass(name = "JointModel")]
pub struct PyJointWrapper {
    pub inner: JointWrapper,
}

#[pymethods]
impl PyJointWrapper {
    #[getter]
    pub fn joint_type(&self) -> JointType {
        self.inner.get_joint_type()
    }

    #[getter]
    pub fn nq(&self) -> usize {
        self.inner.nq()
    }

    #[getter]
    pub fn nv(&self) -> usize {
        self.inner.nv()
    }
}
