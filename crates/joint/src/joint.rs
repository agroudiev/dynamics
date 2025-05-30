//! Trait for joints and a wrapper type for dynamic dispatch.

use crate::data::JointDataWrapper;
use nalgebra::Vector3;
use pyo3::prelude::*;
use rand::rngs::ThreadRng;

/// A wrapper type for the Shape trait to allow dynamic dispatch.
pub type JointWrapper = Box<dyn Joint + Send + Sync>;

/// Joint trait for defining joints in a robotic system.
pub trait Joint {
    /// Returns the joint type.
    fn get_joint_type(&self) -> JointType;

    /// Clones the joint and returns a boxed version of it.
    fn clone_box(&self) -> JointWrapper;

    /// Returns the number of position variables.
    fn nq(&self) -> usize;

    /// Returns the number of velocity variables.
    fn nv(&self) -> usize;

    /// Returns the neutral configuration of the joint.
    fn neutral(&self) -> Vec<f64>;

    /// Creates the joint data.
    fn create_joint_data(&self) -> JointDataWrapper;

    /// Returns the axis of the joint, if applicable.
    fn get_axis(&self) -> Option<Vector3<f64>> {
        None
    }

    /// Returns a random configuration for the joint.
    fn random_configuration(&self, rng: &mut ThreadRng) -> Vec<f64>;
}

/// Enum representing the type of joint.
#[pyclass]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum JointType {
    Fixed,
    Revolute,
}

/// A Python wrapper for the JointWrapper type.
#[pyo3::pyclass]
pub struct PyJointWrapper {
    pub inner: JointWrapper,
}
