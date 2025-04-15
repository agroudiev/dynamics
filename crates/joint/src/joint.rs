//! This module defines a trait for joints and a wrapper type for dynamic dispatch.

use pyo3::prelude::*;

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
}

#[pyclass]
#[derive(Clone, Copy, Debug)]
pub enum JointType {
    Revolute,
}

/// A Python wrapper for the JointWrapper type.
#[pyo3::pyclass]
pub struct PyJointWrapper {
    pub inner: JointWrapper,
}
