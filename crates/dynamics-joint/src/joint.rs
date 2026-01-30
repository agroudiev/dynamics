//! Defines a generic joint model trait and a struct to wrap different joint types.
//!
//! This module provides both:
//! * The `JointModel` trait, which defines the common interface for different joint types.
//! * The `JointWrapper` struct, which encapsulates different joint model implementations and provides a
//!   unified interface to interact with them.

use crate::{
    continuous::JointModelContinuous, fixed::JointModelFixed, joint_data::JointDataWrapper,
    prismatic::JointModelPrismatic, revolute::JointModelRevolute,
};
use dynamics_spatial::{configuration::Configuration, motion::SpatialMotion, se3::SE3};
use rand::rngs::ThreadRng;

#[derive(Clone, Debug)]
/// Enum encapsulating different joint model implementations.
///
/// It serves as the inner representation for the `JointWrapper` struct.
/// As such, users should interact with joints through the `JointWrapper` interface,
/// and avoid using this enum directly.
enum JointModelImpl {
    Continuous(JointModelContinuous),
    Prismatic(JointModelPrismatic),
    Revolute(JointModelRevolute),
    Fixed(JointModelFixed),
}

#[derive(Clone, Debug)]
/// Wrapper struct for different joint model implementations.
///
/// This struct provides a unified interface to interact with different joint types
/// through the `JointModel` trait. It serves as the main entry point for users of
/// the library to work with joints.
pub struct JointWrapper {
    inner: JointModelImpl,
}

impl JointWrapper {
    /// Creates a new `JointWrapper` from a `JointModelContinuous`.
    pub fn continuous(joint: JointModelContinuous) -> Self {
        JointWrapper {
            inner: JointModelImpl::Continuous(joint),
        }
    }

    /// Creates a new `JointWrapper` from a `JointModelPrismatic`.
    pub fn prismatic(joint: JointModelPrismatic) -> Self {
        JointWrapper {
            inner: JointModelImpl::Prismatic(joint),
        }
    }

    /// Creates a new `JointWrapper` from a `JointModelRevolute`.
    pub fn revolute(joint: JointModelRevolute) -> Self {
        JointWrapper {
            inner: JointModelImpl::Revolute(joint),
        }
    }

    /// Creates a new `JointWrapper` from a `JointModelFixed`.
    pub fn fixed(joint: JointModelFixed) -> Self {
        JointWrapper {
            inner: JointModelImpl::Fixed(joint),
        }
    }
}

// The following is boilerplate to forward JointModel trait methods to the inner joint model.
impl JointModel for JointWrapper {
    fn get_joint_type(&self) -> JointType {
        match &self.inner {
            JointModelImpl::Continuous(joint) => joint.get_joint_type(),
            JointModelImpl::Prismatic(joint) => joint.get_joint_type(),
            JointModelImpl::Revolute(joint) => joint.get_joint_type(),
            JointModelImpl::Fixed(joint) => joint.get_joint_type(),
        }
    }

    fn nq(&self) -> usize {
        match &self.inner {
            JointModelImpl::Continuous(joint) => joint.nq(),
            JointModelImpl::Prismatic(joint) => joint.nq(),
            JointModelImpl::Revolute(joint) => joint.nq(),
            JointModelImpl::Fixed(joint) => joint.nq(),
        }
    }

    fn nv(&self) -> usize {
        match &self.inner {
            JointModelImpl::Continuous(joint) => joint.nv(),
            JointModelImpl::Prismatic(joint) => joint.nv(),
            JointModelImpl::Revolute(joint) => joint.nv(),
            JointModelImpl::Fixed(joint) => joint.nv(),
        }
    }

    fn neutral(&self) -> Configuration {
        match &self.inner {
            JointModelImpl::Continuous(joint) => joint.neutral(),
            JointModelImpl::Prismatic(joint) => joint.neutral(),
            JointModelImpl::Revolute(joint) => joint.neutral(),
            JointModelImpl::Fixed(joint) => joint.neutral(),
        }
    }

    fn create_joint_data(&self) -> JointDataWrapper {
        match &self.inner {
            JointModelImpl::Continuous(joint) => joint.create_joint_data(),
            JointModelImpl::Prismatic(joint) => joint.create_joint_data(),
            JointModelImpl::Revolute(joint) => joint.create_joint_data(),
            JointModelImpl::Fixed(joint) => joint.create_joint_data(),
        }
    }

    fn random_configuration(&self, rng: &mut ThreadRng) -> Configuration {
        match &self.inner {
            JointModelImpl::Continuous(joint) => joint.random_configuration(rng),
            JointModelImpl::Prismatic(joint) => joint.random_configuration(rng),
            JointModelImpl::Revolute(joint) => joint.random_configuration(rng),
            JointModelImpl::Fixed(joint) => joint.random_configuration(rng),
        }
    }

    fn transform(&self, q: &Configuration) -> SE3 {
        match &self.inner {
            JointModelImpl::Continuous(joint) => joint.transform(q),
            JointModelImpl::Prismatic(joint) => joint.transform(q),
            JointModelImpl::Revolute(joint) => joint.transform(q),
            JointModelImpl::Fixed(joint) => joint.transform(q),
        }
    }

    fn get_axis(&self) -> Vec<SpatialMotion> {
        match &self.inner {
            JointModelImpl::Continuous(joint) => joint.get_axis(),
            JointModelImpl::Prismatic(joint) => joint.get_axis(),
            JointModelImpl::Revolute(joint) => joint.get_axis(),
            JointModelImpl::Fixed(joint) => joint.get_axis(),
        }
    }
}

/// Joint trait for defining joints in a robotic system.
///
/// This trait provides a common interface for different joint types,
/// allowing for polymorphic behavior when working with various joint models.
pub trait JointModel {
    /// Returns the joint type.
    fn get_joint_type(&self) -> JointType;

    /// Returns the number of position variables.
    fn nq(&self) -> usize;

    /// Returns the number of velocity variables.
    fn nv(&self) -> usize;

    /// Returns the neutral configuration of the joint.
    fn neutral(&self) -> Configuration;

    /// Creates the joint data.
    fn create_joint_data(&self) -> JointDataWrapper;

    /// Returns the axis of the joint, if applicable.
    fn get_axis(&self) -> Vec<SpatialMotion>; // TODO: modify signature

    /// Returns a random configuration for the joint.
    fn random_configuration(&self, rng: &mut ThreadRng) -> Configuration;

    /// Computes the transformation matrix of the joint given its configuration. Featherstone calls it `jcalc`.
    fn transform(&self, q: &Configuration) -> SE3;
}

/// Enum representing the type of joint.
#[cfg_attr(feature = "python", pyo3::prelude::pyclass)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum JointType {
    Continuous,
    Fixed,
    Prismatic,
    Revolute,
}
