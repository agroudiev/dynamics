//! Continuous joint, constraining two objects to rotate around a given axis, without limits.
//!
//! This can be seen as a revolute joint without limits.
//! On top of that, the parametrization of the configuration is different:
//! instead of using an angle $\theta \in [-\pi, \pi]$, continuous joints use
//! unit circle parametrization $(\cos(\theta), \sin(\theta))$.

use dynamics_spatial::{
    configuration::Configuration,
    force::SpatialForce,
    motion::{SpatialMotion, SpatialRotation},
    se3::SE3,
    vector3d::Vector3D,
};
use rand::Rng;

use crate::{
    joint::{JointModel, JointType, JointWrapper},
    joint_data::{JointData, JointDataWrapper, JointError},
    limits::JointLimits,
};

/// Model of a continuous joint.
///
/// This joint constraints two objects to rotate around a given axis, without limits.
#[derive(Clone, Debug)]
pub struct JointModelContinuous {
    /// The axis of rotation expressed in the local frame of the joint.
    pub axis: Vector3D,
    /// The joint limits.
    pub limits: JointLimits,
}

impl JointModelContinuous {
    /// Creates a new `JointModelContinuous` with the given axis of rotation and unbounded limits.
    ///
    /// # Arguments
    ///
    /// * `axis` - The axis of rotation expressed in the local frame of the joint.
    ///
    /// # Returns
    /// A new `JointModelContinuous` object.
    #[must_use]
    pub fn new(axis: Vector3D) -> Self {
        let mut limits = JointLimits::new_unbounded(2);
        limits.min_configuration[0] = -1.01;
        limits.max_configuration[0] = 1.01;
        limits.min_configuration[1] = -1.01;
        limits.max_configuration[1] = 1.01;

        JointModelContinuous { axis, limits }
    }

    /// Creates a new continuous joint model with `x` as axis of rotation.
    ///
    /// # Returns
    /// A new `JointModelContinuous` object.
    #[must_use]
    pub fn new_rux() -> Self {
        Self::new(Vector3D::x())
    }

    /// Creates a new continuous joint model with `y` as axis of rotation.
    ///
    /// # Returns
    /// A new `JointModelContinuous` object.
    #[must_use]
    pub fn new_ruy() -> Self {
        Self::new(Vector3D::y())
    }

    /// Creates a new continuous joint model with `z` as axis of rotation.
    ///
    /// # Returns
    /// A new `JointModelContinuous` object.
    #[must_use]
    pub fn new_ruz() -> Self {
        Self::new(Vector3D::z())
    }
}

impl JointModel for JointModelContinuous {
    fn get_joint_type(&self) -> JointType {
        JointType::Continuous
    }

    fn nq(&self) -> usize {
        2
    }

    fn nv(&self) -> usize {
        1
    }

    fn neutral(&self) -> Configuration {
        Configuration::from_row_slice(&[1.0, 0.0])
    }

    fn get_axis(&self) -> Vec<SpatialMotion> {
        vec![SpatialMotion::from_rotational_axis(&self.axis)]
    }

    fn create_joint_data(&self) -> crate::joint_data::JointDataWrapper {
        JointDataWrapper::continuous(JointDataContinuous::new(self))
    }

    fn random_configuration(&self, rng: &mut rand::rngs::ThreadRng) -> Configuration {
        let angle: f64 = rng.random_range(0.0..(2.0 * std::f64::consts::PI));
        Configuration::from_row_slice(&[angle.cos(), angle.sin()])
    }

    fn transform(&self, q: &Configuration) -> SE3 {
        assert_eq!(
            q.len(),
            2,
            "Continuous joint model expects two values (cosine and sine)."
        );
        let cos = q[0];
        let sin = q[1];
        let angle = sin.atan2(cos);

        SE3::from_parts(
            Vector3D::zeros(),
            SpatialRotation::from_axis_angle(&self.axis, angle),
        )
    }

    fn subspace(&self, v: &Configuration) -> SpatialMotion {
        assert_eq!(
            v.len(),
            1,
            "Continuous joint model expects a single velocity value."
        );
        v[0] * SpatialMotion::from_rotational_axis(&self.axis)
    }

    fn subspace_dual(&self, f: &SpatialForce) -> Configuration {
        Configuration::from_row_slice(&[f.rotation().dot(&self.axis)])
    }

    fn bias(&self) -> SpatialMotion {
        SpatialMotion::zero()
    }
}

/// Data structure containing the mutable properties of a continuous joint.
#[derive(Debug, Clone)]
pub struct JointDataContinuous {
    /// The joint configuration vector (cosine and sine of the angle).
    pub joint_q: Configuration,
    /// The joint velocity vector (angular velocity).
    pub joint_v: Configuration,
    /// The placement of the joint in the local frame.
    pub placement: SE3,
    /// The joint velocity as a spatial motion.
    pub joint_velocity: SpatialMotion,
}

impl Default for JointDataContinuous {
    fn default() -> Self {
        JointDataContinuous {
            joint_q: Configuration::from_row_slice(&[1.0, 0.0]),
            joint_v: Configuration::from_row_slice(&[0.0]),
            placement: SE3::identity(),
            joint_velocity: SpatialMotion::zero(),
        }
    }
}

impl JointDataContinuous {
    /// Creates a new `JointDataContinuous` object.
    ///
    /// # Arguments
    ///
    /// * `model` - The continuous joint model.
    ///
    /// # Returns
    /// A new `JointDataContinuous` object.
    #[must_use]
    pub fn new(_joint_model: &JointModelContinuous) -> Self {
        JointDataContinuous::default()
    }

    /// Returns the cosine of the joint angle.
    #[must_use]
    pub fn cos(&self) -> f64 {
        self.joint_q[0]
    }

    /// Returns the sine of the joint angle.
    #[must_use]
    pub fn sin(&self) -> f64 {
        self.joint_q[1]
    }
}

impl JointData for JointDataContinuous {
    fn get_joint_q(&self) -> &Configuration {
        &self.joint_q
    }

    fn get_joint_v(&self) -> &Configuration {
        &self.joint_v
    }

    fn update(
        &mut self,
        joint_model: &JointWrapper,
        joint_q: &Configuration,
        joint_v: Option<&Configuration>,
    ) -> Result<(), JointError> {
        // TODO: optimize this method to avoid computing the angle

        assert_eq!(
            joint_q.len(),
            2,
            "Continuous joint model expects two values (cosine and sine)."
        );
        if let Some(joint_v) = joint_v {
            assert_eq!(
                joint_v.len(),
                1,
                "Continuous joint model expects a single velocity value."
            );
        }

        // store q and v
        self.joint_q = joint_q.clone();
        if let Some(joint_v) = joint_v {
            self.joint_v = joint_v.clone();
            self.joint_velocity = self.joint_v[0] * joint_model.get_axis()[0].clone();
        }

        // compute angle from cosine and sine
        let angle = self.sin().atan2(self.cos());

        // get axis
        let axis = match joint_model.get_axis().len() {
            1 => &joint_model.get_axis()[0],
            _ => return Err(JointError::MissingAttributeError("axis".to_string())),
        };

        // compute placement
        let rot = SpatialRotation::from_axis_angle(&axis.rotation(), angle);
        self.placement = rot.to_se3(&Vector3D::zeros());

        Ok(())
    }

    fn get_joint_placement(&self) -> SE3 {
        self.placement
    }

    fn get_joint_velocity(&self) -> &SpatialMotion {
        &self.joint_velocity
    }
}
