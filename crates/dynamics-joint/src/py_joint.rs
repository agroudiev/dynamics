use pyo3::prelude::*;

use crate::{
    continuous::JointModelContinuous,
    fixed::JointModelFixed,
    joint::{JointModel, JointType, JointWrapper},
    prismatic::JointModelPrismatic,
    revolute::JointModelRevolute,
};

/// A Python wrapper for the `JointWrapper` type.
#[pyclass(name = "JointModel")]
pub struct PyJointWrapper {
    pub inner: JointWrapper,
}

#[pymethods]
impl PyJointWrapper {
    #[getter]
    #[must_use]
    pub fn joint_type(&self) -> JointType {
        self.inner.get_joint_type()
    }

    #[getter]
    #[must_use]
    pub fn nq(&self) -> usize {
        self.inner.nq()
    }

    #[getter]
    #[must_use]
    pub fn nv(&self) -> usize {
        self.inner.nv()
    }
}

/// A Python wrapper for the `JointModelFixed` struct.
#[pyclass(name = "JointModelFixed")]
pub struct PyJointModelFixed {
    pub inner: JointModelFixed,
}

/// Creates a new continuous joint model with `x` as axis of rotation.
#[pyfunction(name = "JointModelRUBX")]
#[must_use]
pub fn new_rubx() -> PyJointWrapper {
    PyJointWrapper {
        inner: JointWrapper::continuous(JointModelContinuous::new_rux()),
    }
}

/// Creates a new continuous joint model with `y` as axis of rotation.
#[pyfunction(name = "JointModelRUBY")]
#[must_use]
pub fn new_ruby() -> PyJointWrapper {
    PyJointWrapper {
        inner: JointWrapper::continuous(JointModelContinuous::new_ruy()),
    }
}

/// Creates a new continuous joint model with `z` as axis of rotation.
#[pyfunction(name = "JointModelRUBZ")]
#[must_use]
pub fn new_rubz() -> PyJointWrapper {
    PyJointWrapper {
        inner: JointWrapper::continuous(JointModelContinuous::new_ruz()),
    }
}

/// Creates a new prismatic joint model with `x` as axis of rotation.
#[pyfunction(name = "JointModelPX")]
#[must_use]
pub fn new_px() -> PyJointWrapper {
    PyJointWrapper {
        inner: JointWrapper::prismatic(JointModelPrismatic::new_px()),
    }
}

/// Creates a new prismatic joint model with `y` as axis of rotation.
#[pyfunction(name = "JointModelPY")]
#[must_use]
pub fn new_py() -> PyJointWrapper {
    PyJointWrapper {
        inner: JointWrapper::prismatic(JointModelPrismatic::new_py()),
    }
}

/// Creates a new prismatic joint model with `z` as axis of rotation.
#[pyfunction(name = "JointModelPZ")]
#[must_use]
pub fn new_pz() -> PyJointWrapper {
    PyJointWrapper {
        inner: JointWrapper::prismatic(JointModelPrismatic::new_pz()),
    }
}

/// Creates a new revolute joint model with `x` as axis of rotation.
#[pyfunction(name = "JointModelRX")]
#[must_use]
pub fn new_rx() -> PyJointWrapper {
    PyJointWrapper {
        inner: JointWrapper::revolute(JointModelRevolute::new_rx()),
    }
}

/// Creates a new revolute joint model with `y` as axis of rotation.
#[pyfunction(name = "JointModelRY")]
#[must_use]
pub fn new_ry() -> PyJointWrapper {
    PyJointWrapper {
        inner: JointWrapper::revolute(JointModelRevolute::new_ry()),
    }
}

/// Creates a new revolute joint model with `z` as axis of rotation.
#[pyfunction(name = "JointModelRZ")]
#[must_use]
pub fn new_rz() -> PyJointWrapper {
    PyJointWrapper {
        inner: JointWrapper::revolute(JointModelRevolute::new_rz()),
    }
}
