//! Structure containing the mutable properties of a joint.

use crate::{
    continuous::JointDataContinuous, fixed::JointDataFixed, joint::JointWrapper,
    prismatic::JointDataPrismatic, revolute::JointDataRevolute,
};
use dynamics_spatial::{configuration::Configuration, motion::SpatialMotion, se3::SE3};

#[derive(Clone, Debug)]
/// Enum encapsulating different joint data implementations.
///
/// It serves as the inner representation for the `JointDataWrapper` struct.
/// As such, users should interact with joints data through the `JointDataWrapper` interface,
/// and avoid using this enum directly.
enum JointDataImpl {
    Continuous(JointDataContinuous),
    Prismatic(JointDataPrismatic),
    Revolute(JointDataRevolute),
    Fixed(JointDataFixed),
}

#[derive(Clone, Debug)]
/// Wrapper struct for different joint data implementations.
///
/// This struct provides a unified interface to interact with different joint types
/// through the `JointData` trait. It serves as the main entry point for users of
/// the library to work with joints data.
pub struct JointDataWrapper {
    inner: JointDataImpl,
}

/// Trait for joint data, providing methods to access and update joint properties.
pub trait JointData {
    /// Returns the joint configuration vector.
    fn get_joint_q(&self) -> &Configuration;

    /// Returns the joint velocity vector.
    fn get_joint_v(&self) -> &Configuration;

    /// Returns the placement of the joint in the world frame.
    fn get_joint_placement(&self) -> SE3;

    /// Updates the joint data with the current position and velocity configurations.
    fn update(
        &mut self,
        joint_model: &JointWrapper,
        joint_q: &Configuration,
        joint_v: Option<&Configuration>,
    );

    /// Returns the joint velocity as a spatial motion.
    fn get_joint_velocity(&self) -> &SpatialMotion;
}

impl JointDataWrapper {
    /// Creates a new `JointDataWrapper` from a `JointDataContinuous`.
    pub fn continuous(joint_data: JointDataContinuous) -> Self {
        JointDataWrapper {
            inner: JointDataImpl::Continuous(joint_data),
        }
    }

    /// Creates a new `JointDataWrapper` from a `JointDataPrismatic`.
    pub fn prismatic(joint_data: JointDataPrismatic) -> Self {
        JointDataWrapper {
            inner: JointDataImpl::Prismatic(joint_data),
        }
    }

    /// Creates a new `JointDataWrapper` from a `JointDataRevolute`.
    pub fn revolute(joint_data: JointDataRevolute) -> Self {
        JointDataWrapper {
            inner: JointDataImpl::Revolute(joint_data),
        }
    }

    /// Creates a new `JointDataWrapper` from a `JointDataFixed`.
    pub fn fixed(joint_data: JointDataFixed) -> Self {
        JointDataWrapper {
            inner: JointDataImpl::Fixed(joint_data),
        }
    }
}

// TODO: use macros to reduce boilerplate
impl JointData for JointDataWrapper {
    fn get_joint_q(&self) -> &Configuration {
        match &self.inner {
            JointDataImpl::Continuous(joint_data) => joint_data.get_joint_q(),
            JointDataImpl::Prismatic(joint_data) => joint_data.get_joint_q(),
            JointDataImpl::Revolute(joint_data) => joint_data.get_joint_q(),
            JointDataImpl::Fixed(joint_data) => joint_data.get_joint_q(),
        }
    }

    fn get_joint_v(&self) -> &Configuration {
        match &self.inner {
            JointDataImpl::Continuous(joint_data) => joint_data.get_joint_v(),
            JointDataImpl::Prismatic(joint_data) => joint_data.get_joint_v(),
            JointDataImpl::Revolute(joint_data) => joint_data.get_joint_v(),
            JointDataImpl::Fixed(joint_data) => joint_data.get_joint_v(),
        }
    }

    fn get_joint_placement(&self) -> SE3 {
        match &self.inner {
            JointDataImpl::Continuous(joint_data) => joint_data.get_joint_placement(),
            JointDataImpl::Prismatic(joint_data) => joint_data.get_joint_placement(),
            JointDataImpl::Revolute(joint_data) => joint_data.get_joint_placement(),
            JointDataImpl::Fixed(joint_data) => joint_data.get_joint_placement(),
        }
    }

    fn update(
        &mut self,
        joint_model: &JointWrapper,
        joint_q: &Configuration,
        joint_v: Option<&Configuration>,
    ) {
        match &mut self.inner {
            JointDataImpl::Continuous(joint_data) => {
                joint_data.update(joint_model, joint_q, joint_v)
            }
            JointDataImpl::Prismatic(joint_data) => {
                joint_data.update(joint_model, joint_q, joint_v)
            }
            JointDataImpl::Revolute(joint_data) => joint_data.update(joint_model, joint_q, joint_v),
            JointDataImpl::Fixed(joint_data) => joint_data.update(joint_model, joint_q, joint_v),
        }
    }

    fn get_joint_velocity(&self) -> &SpatialMotion {
        match &self.inner {
            JointDataImpl::Continuous(joint_data) => joint_data.get_joint_velocity(),
            JointDataImpl::Prismatic(joint_data) => joint_data.get_joint_velocity(),
            JointDataImpl::Revolute(joint_data) => joint_data.get_joint_velocity(),
            JointDataImpl::Fixed(joint_data) => joint_data.get_joint_velocity(),
        }
    }
}
