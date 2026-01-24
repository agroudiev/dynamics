//! Prismatic joint, constraining two objects to translate along a given axis.

use dynamics_spatial::{
    configuration::Configuration,
    motion::{SpatialMotion, SpatialRotation},
    se3::SE3,
    vector3d::Vector3D,
};
use rand::rngs::ThreadRng;

use crate::{
    data::{JointData, JointDataWrapper, JointError},
    joint::{JointModel, JointType, JointWrapper},
    limits::JointLimits,
};

/// Model of a prismatic joint.
///
/// This joint constraints two objects to translate along a given axis.
#[derive(Clone, Debug)]
pub struct JointModelPrismatic {
    /// The axis of translation expressed in the local frame of the joint.
    pub axis: Vector3D,
    /// The joint limits.
    pub limits: JointLimits,
}

impl JointModelPrismatic {
    /// Creates a new `JointModelPrismatic` with the given axis of translation and unbounded limits.
    ///
    /// # Arguments
    ///
    /// * `axis` - The axis of translation expressed in the local frame of the joint.
    ///
    /// # Returns
    /// A new `JointModelPrismatic` object.
    #[must_use]
    pub fn new(axis: Vector3D) -> Self {
        JointModelPrismatic {
            axis,
            limits: JointLimits::new_unbounded(1),
        }
    }

    /// Creates a new prismatic joint model with `x` as axis of translation.
    ///
    /// # Returns
    /// A new `JointModelPrismatic` object.
    #[must_use]
    pub fn new_px() -> Self {
        Self::new(Vector3D::x())
    }

    /// Creates a new prismatic joint model with `y` as axis of translation.
    ///
    /// # Returns
    /// A new `JointModelPrismatic` object.
    #[must_use]
    pub fn new_py() -> Self {
        Self::new(Vector3D::y())
    }

    /// Creates a new prismatic joint model with `z` as axis of translation.
    ///
    /// # Returns
    /// A new `JointModelPrismatic` object.
    #[must_use]
    pub fn new_pz() -> Self {
        Self::new(Vector3D::z())
    }
}

impl JointModel for JointModelPrismatic {
    fn get_joint_type(&self) -> JointType {
        JointType::Prismatic
    }

    fn clone_box(&self) -> JointWrapper {
        Box::new(self.clone())
    }

    fn nq(&self) -> usize {
        1
    }

    fn nv(&self) -> usize {
        1
    }

    fn neutral(&self) -> Configuration {
        Configuration::from_row_slice(&[0.0])
    }

    fn create_joint_data(&self) -> JointDataWrapper {
        Box::new(JointDataPrismatic::new(self))
    }

    fn get_axis(&self) -> Vec<SpatialMotion> {
        vec![SpatialMotion::from_translational_axis(&self.axis)]
    }

    fn random_configuration(&self, rng: &mut ThreadRng) -> Configuration {
        Configuration::random(
            1,
            rng,
            &self.limits.min_configuration,
            &self.limits.max_configuration,
        )
    }

    fn transform(&self, q: &Configuration) -> SE3 {
        SE3::from_parts(self.axis * q[0], SpatialRotation::identity())
    }
}

/// Data associated to a prismatic joint.
#[derive(Clone, Debug)]
pub struct JointDataPrismatic {
    // The current position of the joint.
    pub q: f64,
    /// The placement of the joint in the local frame.
    pub placement: SE3,
}

impl JointDataPrismatic {
    /// Creates a new `JointDataPrismatic` associated to the given joint model.
    ///
    /// # Arguments
    ///
    /// * `model` - The prismatic joint model.
    ///
    /// # Returns
    /// A new `JointDataPrismatic` object.
    #[must_use]
    pub fn new(_model: &JointModelPrismatic) -> Self {
        JointDataPrismatic {
            q: 0.0,
            placement: SE3::identity(),
        }
    }
}

impl JointData for JointDataPrismatic {
    fn get_joint_placement(&self) -> SE3 {
        self.placement
    }

    fn update(
        &mut self,
        joint_model: &JointWrapper,
        q_joint: &Configuration,
    ) -> Result<(), JointError> {
        assert_eq!(
            q_joint.len(),
            1,
            "Prismatic joint data update expects a single position value."
        );

        let axis = match joint_model.get_axis().len() {
            1 => &joint_model.get_axis()[0],
            _ => return Err(JointError::MissingAttributeError("axis".to_string())),
        };

        self.placement =
            SE3::from_parts(axis.translation() * q_joint[0], SpatialRotation::identity());
        Ok(())
    }
}
