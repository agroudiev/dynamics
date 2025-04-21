//! Revolute joint, constraining two objects to rotate around a given axis.

use crate::{
    data::{JointData, JointDataWrapper, JointError},
    joint::{Joint, JointType, JointWrapper},
};
use nalgebra::{IsometryMatrix3, Rotation3, Translation, Vector3};
use pyo3::prelude::*;

/// Model of a revolute joint.
///
/// This joint constraints two objects to rotate around a given axis.
#[derive(Clone, Debug)]
pub struct JointModelRevolute {
    /// The axis of rotation expressed in the local frame of the joint.
    pub axis: Vector3<f64>,
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
}

/// Data structure containing the mutable properties of a revolute joint.
#[derive(Default, Debug)]
pub struct JointDataRevolute {
    /// The angle of rotation.
    pub q: f64,
    /// The placement of the joint in the local frame.
    pub placement: IsometryMatrix3<f64>,
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
        data.update(joint_model, 0.0).unwrap();
        data
    }
}

impl JointData for JointDataRevolute {
    fn get_joint_placement(&self) -> IsometryMatrix3<f64> {
        self.placement
    }

    fn update(&mut self, joint_model: &dyn Joint, q: f64) -> Result<(), JointError> {
        self.q = q;
        let axis = match joint_model.get_axis() {
            Some(axis) => axis,
            None => return Err(JointError::MissingAttributeError("axis".to_string())),
        };

        self.placement = IsometryMatrix3::from_parts(
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
    PyJointModelRevolute {
        inner: JointModelRevolute {
            axis: *Vector3::x_axis(),
        },
    }
}

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;

    use super::*;

    #[test]
    fn test_joint_model_revolute() {
        let joint = JointModelRevolute {
            axis: *Vector3::x_axis(),
        };
        assert_eq!(joint.get_joint_type(), JointType::Revolute);
        assert_eq!(joint.nq(), 1);
        assert_eq!(joint.nv(), 1);
        assert_eq!(joint.neutral(), vec![0.0]);
    }

    #[test]
    fn test_joint_data_revolute_xaxis() {
        let joint_model = JointModelRevolute {
            axis: *Vector3::x_axis(),
        };
        let mut joint_data = joint_model.create_joint_data();
        let q = 1.0;
        joint_data.update(&joint_model, q).unwrap();

        assert_relative_eq!(joint_data.get_joint_placement().rotation.angle(), q);
        assert_eq!(
            joint_data.get_joint_placement().translation.vector,
            Vector3::zeros()
        );
    }
}
