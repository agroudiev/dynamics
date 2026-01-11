//! Revolute joint, constraining two objects to rotate around a given axis.

use std::f64::consts::PI;

use crate::{
    data::{JointData, JointDataWrapper, JointError},
    joint::{JointModel, JointType, JointWrapper},
};
use pyo3::prelude::*;
use rand::Rng;
use spatial::{
    configuration::Configuration,
    motion::{SpatialMotion, SpatialRotation},
    se3::SE3,
    vector3d::Vector3D,
};

/// Model of a revolute joint.
///
/// This joint constraints two objects to rotate around a given axis.
#[derive(Clone, Debug)]
pub struct JointModelRevolute {
    /// The axis of rotation expressed in the local frame of the joint.
    pub axis: Vector3D,
    /// The lower limit of the joint angle.
    pub lower_limit: f64,
    /// The upper limit of the joint angle.
    pub upper_limit: f64,
    /// The effort limit of the joint.
    pub effort_limit: f64,
    /// The velocity limit of the joint.
    pub velocity_limit: f64,
}

impl JointModelRevolute {
    /// Creates a new `JointModelRevolute` with the given axis of rotation.
    ///
    /// # Arguments
    ///
    /// * `axis` - The axis of rotation expressed in the local frame of the joint.
    ///
    /// # Returns
    /// A new `JointModelRevolute` object.
    pub fn new(axis: Vector3D) -> Self {
        JointModelRevolute {
            axis,
            lower_limit: f64::NEG_INFINITY,
            upper_limit: f64::INFINITY,
            effort_limit: f64::INFINITY,
            velocity_limit: f64::INFINITY,
        }
    }
}

impl JointModel for JointModelRevolute {
    fn get_joint_type(&self) -> JointType {
        JointType::Revolute
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
        Configuration::zeros(1)
    }

    fn create_joint_data(&self) -> JointDataWrapper {
        Box::new(JointDataRevolute::new(self))
    }

    fn get_axis(&self) -> Vec<SpatialMotion> {
        vec![SpatialMotion::from_axis(&self.axis)]
    }

    fn random_configuration(&self, rng: &mut rand::rngs::ThreadRng) -> Configuration {
        let q = rng.random_range(self.lower_limit.max(-PI)..self.upper_limit.min(PI));
        Configuration::from_row_slice(&[q])
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
#[derive(Default, Debug)]
pub struct JointDataRevolute {
    /// The angle of rotation.
    pub q: f64,
    /// The placement of the joint in the local frame.
    pub placement: SE3,
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
    pub fn new(joint_model: &JointModelRevolute) -> Self {
        let mut data = JointDataRevolute::default();
        // safe since we just created a revolute joint model
        // and we know that a revolute joint has an axis
        let joint_model_box: JointWrapper = Box::new(joint_model.clone());
        data.update(&joint_model_box, &Configuration::zeros(1))
            .unwrap();
        data
    }
}

impl JointData for JointDataRevolute {
    fn get_joint_placement(&self) -> SE3 {
        self.placement
    }

    fn update(&mut self, joint_model: &JointWrapper, q: &Configuration) -> Result<(), JointError> {
        assert_eq!(q.len(), 1, "Revolute joint model expects a single angle.");
        let q = q[0];
        self.q = q;
        let axis = match joint_model.get_axis().len() {
            1 => &joint_model.get_axis()[0],
            _ => return Err(JointError::MissingAttributeError("axis".to_string())),
        };

        let rot = SpatialRotation::from_axis_angle(&axis.rotation(), q);
        self.placement = rot.to_se3(&Vector3D::zeros());
        Ok(())
    }
}

/// A Python wrapper for the `JointModelRevolute` struct.
#[pyclass(name = "JointModelRevolute")]
pub struct PyJointModelRevolute {
    pub inner: JointModelRevolute,
}

/// Creates a new revolute joint model with `x` as axis of rotation.
#[pyfunction(name = "JointModelRX")]
pub fn new_joint_model_revolute_x() -> PyJointModelRevolute {
    let inner = JointModelRevolute::new(Vector3D::new(1.0, 0.0, 0.0));
    PyJointModelRevolute { inner }
}

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;

    use super::*;

    #[test]
    fn test_joint_model_revolute() {
        let joint = JointModelRevolute {
            axis: Vector3D::new(1.0, 0.0, 0.0),
            lower_limit: f64::NEG_INFINITY,
            upper_limit: f64::INFINITY,
            effort_limit: f64::INFINITY,
            velocity_limit: f64::INFINITY,
        };
        assert_eq!(joint.get_joint_type(), JointType::Revolute);
        assert_eq!(joint.nq(), 1);
        assert_eq!(joint.nv(), 1);
        assert_eq!(joint.neutral(), Configuration::zeros(1));
    }

    #[test]
    fn test_joint_data_revolute_xaxis() {
        let joint_model = JointModelRevolute::new(Vector3D::new(1.0, 0.0, 0.0));
        let mut joint_data = joint_model.create_joint_data();
        let q = Configuration::ones(1);
        let joint_model_box: JointWrapper = Box::new(joint_model.clone());
        joint_data.update(&joint_model_box, &q).unwrap();

        assert_relative_eq!(joint_data.get_joint_placement().rotation().angle(), q[0]);
        assert_eq!(
            joint_data.get_joint_placement().translation(),
            Vector3D::zeros()
        );
    }
}
