//! Fixed joint, without any degree of freedom.

use crate::{
    joint::{JointModel, JointType, JointWrapper},
    joint_data::{JointData, JointDataWrapper, JointError},
};
use dynamics_spatial::{configuration::Configuration, motion::SpatialMotion, se3::SE3};
use rand::rngs::ThreadRng;

/// Model of a fixed joint.
#[derive(Clone, Debug, Default)]
pub struct JointModelFixed {}

impl JointModel for JointModelFixed {
    fn get_joint_type(&self) -> JointType {
        JointType::Fixed
    }

    fn nq(&self) -> usize {
        0
    }

    fn nv(&self) -> usize {
        0
    }

    fn neutral(&self) -> Configuration {
        Configuration::zeros(0)
    }

    fn create_joint_data(&self) -> JointDataWrapper {
        JointDataWrapper::fixed(JointDataFixed::new(self))
    }

    fn random_configuration(&self, _rng: &mut ThreadRng) -> Configuration {
        Configuration::zeros(0)
    }

    fn transform(&self, q: &Configuration) -> SE3 {
        assert_eq!(q.len(), 0, "Fixed joint model expects no configuration.");
        SE3::identity()
    }

    fn get_axis(&self) -> Vec<SpatialMotion> {
        Vec::new()
    }

    fn subspace(&self, v: &Configuration) -> SpatialMotion {
        assert_eq!(v.len(), 0, "Fixed joint model expects no velocity.");
        SpatialMotion::zero() // TODO: check
    }
}

/// Data structure containing the mutable properties of a fixed joint.
#[derive(Debug, Clone)]
pub struct JointDataFixed {
    /// The joint configuration vector (always empty).
    pub joint_q: Configuration,
    /// The joint velocity vector (always empty).
    pub joint_v: Configuration,
    /// The placement of the joint in the local frame.
    pub placement: SE3,
    /// The joint velocity as a spatial motion.
    pub joint_velocity: SpatialMotion,
}

impl JointDataFixed {
    /// Creates a new [`JointDataFixed`] from given joint model.
    ///
    /// # Arguments
    ///
    /// * `joint_model` - The fixed joint model.
    ///
    /// # Returns
    /// A new [`JointDataFixed`] object.
    #[must_use]
    pub fn new(_joint_model: &JointModelFixed) -> Self {
        JointDataFixed {
            joint_q: Configuration::zeros(0),
            joint_v: Configuration::zeros(0),
            placement: SE3::identity(),
            joint_velocity: SpatialMotion::zero(),
        }
    }
}

impl JointData for JointDataFixed {
    fn get_joint_q(&self) -> &Configuration {
        &self.joint_q
    }

    fn get_joint_v(&self) -> &Configuration {
        &self.joint_v
    }

    fn get_joint_placement(&self) -> SE3 {
        self.placement
    }

    fn update(
        &mut self,
        _joint_model: &JointWrapper,
        _joint_q: &Configuration,
        _joint_v: Option<&Configuration>,
    ) -> Result<(), JointError> {
        Ok(())
    }

    fn get_joint_velocity(&self) -> &SpatialMotion {
        &self.joint_velocity
    }
}
