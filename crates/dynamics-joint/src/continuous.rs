//! Continuous joint, constraining two objects to rotate around a given axis, without limits.
//!
//! This can be seen as a revolute joint without limits.
//! On top of that, the parametrization of the configuration is different:
//! instead of using an angle $\theta \in [-\pi, \pi]$, continuous joints use
//! unit circle parametrization $(cos(\theta), sin(\theta))$.

use dynamics_spatial::{
    configuration::Configuration,
    motion::{SpatialMotion, SpatialRotation},
    se3::SE3,
    vector3d::Vector3D,
};
use pyo3::prelude::*;
use rand::Rng;

use crate::{
    data::{JointData, JointError},
    joint::{JointModel, JointType, JointWrapper, PyJointWrapper},
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
    pub fn new_rx() -> Self {
        Self::new(Vector3D::x())
    }

    /// Creates a new continuous joint model with `y` as axis of rotation.
    ///
    /// # Returns
    /// A new `JointModelContinuous` object.
    #[must_use]
    pub fn new_ry() -> Self {
        Self::new(Vector3D::y())
    }

    /// Creates a new continuous joint model with `z` as axis of rotation.
    ///
    /// # Returns
    /// A new `JointModelContinuous` object.
    #[must_use]
    pub fn new_rz() -> Self {
        Self::new(Vector3D::z())
    }
}

impl JointModel for JointModelContinuous {
    fn get_joint_type(&self) -> JointType {
        JointType::Continuous
    }

    fn clone_box(&self) -> JointWrapper {
        Box::new(self.clone())
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

    fn create_joint_data(&self) -> crate::data::JointDataWrapper {
        Box::new(JointDataContinuous::new(self))
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
}

/// Data structure containing the mutable properties of a continuous joint.
#[derive(Debug)]
pub struct JointDataContinuous {
    /// The cosine of the angle.
    pub cos: f64,
    /// The sine of the angle.
    pub sin: f64,
    /// The placement of the joint in the local frame.
    pub placement: SE3,
}

impl Default for JointDataContinuous {
    fn default() -> Self {
        JointDataContinuous {
            cos: 1.0,
            sin: 0.0,
            placement: SE3::identity(),
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
    pub fn new(joint_model: &JointModelContinuous) -> Self {
        let mut data = JointDataContinuous::default();
        // safe since we just created a continuous joint model
        // and we know that a continuous joint has an axis
        let joint_model_box: JointWrapper = Box::new(joint_model.clone());
        data.update(&joint_model_box, &Configuration::zeros(2))
            .unwrap();
        data
    }
}

impl JointData for JointDataContinuous {
    fn update(&mut self, joint_model: &JointWrapper, q: &Configuration) -> Result<(), JointError> {
        // TODO: optimize this method to avoid computing the angle

        assert_eq!(
            q.len(),
            2,
            "Continuous joint model expects two values (cosine and sine)."
        );

        // compute angle from cosine and sine
        self.cos = q[0];
        self.sin = q[1];
        let angle = self.sin.atan2(self.cos);

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
}

/// Creates a new revolute joint model with `x` as axis of rotation.
#[pyfunction(name = "JointModelRUX")]
#[must_use]
pub fn new_rux() -> PyJointWrapper {
    PyJointWrapper {
        inner: Box::new(JointModelContinuous::new_rx()),
    }
}

/// Creates a new revolute joint model with `y` as axis of rotation.
#[pyfunction(name = "JointModelRUY")]
#[must_use]
pub fn new_ruy() -> PyJointWrapper {
    PyJointWrapper {
        inner: Box::new(JointModelContinuous::new_ry()),
    }
}

/// Creates a new revolute joint model with `z` as axis of rotation.
#[pyfunction(name = "JointModelRUZ")]
#[must_use]
pub fn new_ruz() -> PyJointWrapper {
    PyJointWrapper {
        inner: Box::new(JointModelContinuous::new_rz()),
    }
}
