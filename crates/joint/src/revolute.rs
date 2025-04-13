use crate::joint::{Joint, JointType, JointWrapper};
use nalgebra::Vector3;
use pyo3::prelude::*;

/// A revolute joint model.
#[derive(Clone, Debug)]
pub struct JointModelRevolute {
    pub axis: Vector3<f32>,
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
}

#[pyclass(name = "JointModelRevolute")]
pub struct PyJointModelRevolute {
    pub inner: JointModelRevolute,
}

#[pyfunction(name = "JointModelRX")]
pub fn new_joint_model_revolute_x() -> PyJointModelRevolute {
    PyJointModelRevolute {
        inner: JointModelRevolute {
            axis: *Vector3::x_axis(),
        },
    }
}
