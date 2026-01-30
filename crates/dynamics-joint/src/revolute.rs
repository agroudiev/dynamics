//! Revolute joint, constraining two objects to rotate around a given axis.

use crate::{
    joint::{JointModel, JointType, JointWrapper},
    joint_data::{JointData, JointDataWrapper, JointError},
    limits::JointLimits,
};
use dynamics_spatial::{
    configuration::Configuration,
    motion::{SpatialMotion, SpatialRotation},
    se3::SE3,
    vector3d::Vector3D,
};
use rand::rngs::ThreadRng;

/// Model of a revolute joint.
///
/// This joint constraints two objects to rotate around a given axis.
#[derive(Clone, Debug)]
pub struct JointModelRevolute {
    /// The axis of rotation expressed in the local frame of the joint.
    pub axis: Vector3D,
    /// The joint limits.
    pub limits: JointLimits,
}

impl JointModelRevolute {
    /// Creates a new `JointModelRevolute` with the given axis of rotation and unbounded limits.
    ///
    /// # Arguments
    ///
    /// * `axis` - The axis of rotation expressed in the local frame of the joint.
    ///
    /// # Returns
    /// A new `JointModelRevolute` object.
    #[must_use]
    pub fn new(axis: Vector3D) -> Self {
        JointModelRevolute {
            axis,
            limits: JointLimits::new_unbounded(1),
        }
    }

    /// Creates a new revolute joint model with `x` as axis of rotation.
    ///
    /// # Returns
    /// A new `JointModelRevolute` object.
    #[must_use]
    pub fn new_rx() -> Self {
        Self::new(Vector3D::x())
    }

    /// Creates a new revolute joint model with `y` as axis of rotation.
    ///
    /// # Returns
    /// A new `JointModelRevolute` object.
    #[must_use]
    pub fn new_ry() -> Self {
        Self::new(Vector3D::y())
    }

    /// Creates a new revolute joint model with `z` as axis of rotation.
    ///
    /// # Returns
    /// A new `JointModelRevolute` object.
    #[must_use]
    pub fn new_rz() -> Self {
        Self::new(Vector3D::z())
    }
}

impl JointModel for JointModelRevolute {
    fn get_joint_type(&self) -> JointType {
        JointType::Revolute
    }

    fn nq(&self) -> usize {
        1
    }

    fn nv(&self) -> usize {
        1
    }

    fn neutral(&self) -> Configuration {
        Configuration::zeros(1)
    }

    fn create_joint_data(&self) -> JointDataWrapper {
        Box::new(JointDataRevolute::new(self))
    }

    fn get_axis(&self) -> Vec<SpatialMotion> {
        vec![SpatialMotion::from_rotational_axis(&self.axis)]
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
        assert_eq!(q.len(), 1, "Revolute joint model expects a single angle.");
        let angle = q[0];
        SE3::from_parts(
            Vector3D::zeros(),
            SpatialRotation::from_axis_angle(&self.axis, angle),
        )
    }
}

/// Data structure containing the mutable properties of a revolute joint.
#[derive(Debug, Clone)]
pub struct JointDataRevolute {
    /// The joint configuration vector (angle of rotation).
    pub joint_q: Configuration,
    /// The joint velocity vector (angle velocity).
    pub joint_v: Configuration,
    /// The placement of the joint in the local frame.
    pub placement: SE3,
    /// The joint velocity as a spatial motion.
    pub joint_velocity: SpatialMotion,
}

impl JointDataRevolute {
    /// Creates a new `JointDataRevolute` from given joint model, with the initial angle set to 0.0.
    ///
    /// # Arguments
    ///
    /// * `joint_model` - The revolute joint model.
    ///
    /// # Returns
    /// A new `JointDataRevolute` object.
    #[must_use]
    pub fn new(_joint_model: &JointModelRevolute) -> Self {
        JointDataRevolute {
            joint_q: Configuration::zeros(1),
            joint_v: Configuration::zeros(1),
            placement: SE3::identity(),
            joint_velocity: SpatialMotion::zero(),
        }
    }
}

impl JointData for JointDataRevolute {
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
        joint_model: &JointWrapper,
        joint_q: &Configuration,
        joint_v: Option<&Configuration>,
    ) -> Result<(), JointError> {
        assert_eq!(
            joint_q.len(),
            1,
            "Revolute joint model expects a single angle."
        );
        if let Some(joint_v) = joint_v {
            assert_eq!(
                joint_v.len(),
                1,
                "Revolute joint model expects a single velocity value."
            );
        }

        // store q and v
        self.joint_q = joint_q.clone();
        if let Some(joint_v) = joint_v {
            self.joint_v = joint_v.clone();
            self.joint_velocity = self.joint_v[0] * joint_model.get_axis()[0].clone();
        }

        let axis = match joint_model.get_axis().len() {
            1 => &joint_model.get_axis()[0],
            _ => return Err(JointError::MissingAttributeError("axis".to_string())),
        };

        let rot = SpatialRotation::from_axis_angle(&axis.rotation(), self.joint_q[0]);
        self.placement = rot.to_se3(&Vector3D::zeros());
        Ok(())
    }

    fn clone_box(&self) -> JointDataWrapper {
        Box::new(self.clone())
    }

    fn get_joint_velocity(&self) -> &SpatialMotion {
        &self.joint_velocity
    }
}

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;

    use super::*;

    #[test]
    fn test_joint_model_revolute() {
        let joint = JointModelRevolute::new(Vector3D::new(1.0, 0.0, 0.0));
        assert_eq!(joint.get_joint_type(), JointType::Revolute);
        assert_eq!(joint.nq(), 1);
        assert_eq!(joint.nv(), 1);
        assert_eq!(joint.neutral(), Configuration::zeros(1));
        let _ = joint.create_joint_data();
        let _ = joint.get_axis();
        let _ = joint.random_configuration(&mut rand::rng());
    }

    #[test]
    fn test_joint_data_revolute_xaxis() {
        let joint_model = JointModelRevolute::new(Vector3D::new(1.0, 0.0, 0.0));
        let mut joint_data = joint_model.create_joint_data();
        let q = Configuration::ones(1);
        joint_data
            .update(&JointWrapper::revolute(joint_model), &q, None)
            .unwrap();

        assert_relative_eq!(joint_data.get_joint_placement().rotation().angle(), q[0]);
        assert_eq!(
            joint_data.get_joint_placement().translation(),
            Vector3D::zeros()
        );
    }
}
