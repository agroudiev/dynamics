//! Revolute joint, constraining two objects to rotate around a given axis.

use std::f64::consts::PI;

use crate::{
    data::{JointData, JointDataWrapper, JointError},
    joint::{Joint, JointType, JointWrapper},
};
use nalgebra::{DVector, Rotation3, Translation, Vector3};
use pyo3::prelude::*;
use rand::Rng;
use spatial::{se3::SE3, transform::SpatialTransform};

/// Model of a revolute joint.
///
/// This joint constraints two objects to rotate around a given axis.
#[derive(Clone, Debug)]
pub struct JointModelRevolute {
    /// The axis of rotation expressed in the local frame of the joint.
    pub axis: Vector3<f64>,
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
    pub fn new(axis: Vector3<f64>) -> Self {
        JointModelRevolute {
            axis,
            lower_limit: f64::NEG_INFINITY,
            upper_limit: f64::INFINITY,
            effort_limit: f64::INFINITY,
            velocity_limit: f64::INFINITY,
        }
    }
}

impl Joint for JointModelRevolute {
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

    fn neutral(&self) -> Vec<f64> {
        vec![0.0]
    }

    fn create_joint_data(&self) -> JointDataWrapper {
        Box::new(JointDataRevolute::new(self))
    }

    fn get_axis(&self) -> Option<Vector3<f64>> {
        Some(self.axis)
    }

    fn random_configuration(&self, rng: &mut rand::rngs::ThreadRng) -> Vec<f64> {
        let q = rng.random_range(self.lower_limit.max(-PI)..self.upper_limit.min(PI));
        vec![q]
    }

    fn transform(&self, q: &nalgebra::DVector<f64>) -> SpatialTransform {
        assert_eq!(q.len(), 1, "Revolute joint model expects a single angle.");
        let angle = q[0];
        let rot = Rotation3::from_axis_angle(&nalgebra::Unit::new_normalize(self.axis), angle);
        SpatialTransform::from_rotation(rot)
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
        data.update(&joint_model_box, &DVector::zeros(1)).unwrap();
        data
    }
}

impl JointData for JointDataRevolute {
    fn get_joint_placement(&self) -> SE3 {
        self.placement
    }

    fn update(&mut self, joint_model: &JointWrapper, q: &DVector<f64>) -> Result<(), JointError> {
        assert_eq!(q.len(), 1, "Revolute joint model expects a single angle.");
        let q = q[0];
        self.q = q;
        let axis = match joint_model.get_axis() {
            Some(axis) => axis,
            None => return Err(JointError::MissingAttributeError("axis".to_string())),
        };

        self.placement = SE3::from_parts(
            Translation::identity(),
            Rotation3::from_axis_angle(&nalgebra::Unit::new_normalize(axis), q),
        );
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
    let inner = JointModelRevolute::new(*Vector3::x_axis());
    PyJointModelRevolute { inner }
}

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;

    use super::*;

    #[test]
    fn test_joint_model_revolute() {
        let joint = JointModelRevolute {
            axis: *Vector3::x_axis(),
            lower_limit: f64::NEG_INFINITY,
            upper_limit: f64::INFINITY,
            effort_limit: f64::INFINITY,
            velocity_limit: f64::INFINITY,
        };
        assert_eq!(joint.get_joint_type(), JointType::Revolute);
        assert_eq!(joint.nq(), 1);
        assert_eq!(joint.nv(), 1);
        assert_eq!(joint.neutral(), vec![0.0]);
    }

    #[test]
    fn test_joint_data_revolute_xaxis() {
        let joint_model = JointModelRevolute::new(*Vector3::x_axis());
        let mut joint_data = joint_model.create_joint_data();
        let q = DVector::from_element(1, 1.0);
        let joint_model_box: JointWrapper = Box::new(joint_model.clone());
        joint_data.update(&joint_model_box, &q).unwrap();

        assert_relative_eq!(joint_data.get_joint_placement().rotation.angle(), q[0]);
        assert_eq!(
            joint_data.get_joint_placement().translation.vector,
            Vector3::zeros()
        );
    }
}
