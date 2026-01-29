use dynamics_spatial::{py_configuration::PyConfiguration, py_se3::PySE3};
use pyo3::{exceptions::PyValueError, prelude::*};

use crate::{joint_data::JointDataWrapper, py_joint::PyJointWrapper};

/// A Python wrapper for the `JointDataWrapper` type.
#[pyclass(name = "JointData")]
pub struct PyJointDataWrapper {
    pub inner: JointDataWrapper,
}

#[pymethods]
impl PyJointDataWrapper {
    #[getter]
    #[must_use]
    /// Returns the joint placement in the world frame.
    pub fn joint_placement(&self) -> PySE3 {
        PySE3 {
            inner: self.inner.get_joint_placement(),
        }
    }

    #[getter]
    #[must_use]
    /// Returns the joint configuration vector.
    pub fn joint_q(&self) -> PyConfiguration {
        PyConfiguration::new(self.inner.get_joint_q().clone())
    }

    #[getter]
    #[must_use]
    /// Returns the joint velocity vector.
    pub fn joint_v(&self) -> PyConfiguration {
        PyConfiguration::new(self.inner.get_joint_v().clone())
    }

    #[getter]
    #[allow(non_snake_case)]
    /// Returns the joint placement in the world frame.
    ///
    /// Alias for `joint_placement` property.
    pub fn M(&self) -> PySE3 {
        PySE3 {
            inner: self.inner.get_joint_placement(),
        }
    }

    #[pyo3(text_signature = "(joint_model, joint_q, joint_v=None)")]
    pub fn update(
        &mut self,
        joint_model: &PyJointWrapper,
        joint_q: &PyConfiguration,
        joint_v: Option<&PyConfiguration>,
    ) -> PyResult<()> {
        match self.inner.update(
            &joint_model.inner,
            joint_q.to_configuration(),
            joint_v.map(|v| v.to_configuration()),
        ) {
            Ok(()) => Ok(()),
            Err(e) => Err(PyValueError::new_err(format!(
                "Failed to update joint data: {e:?}"
            ))),
        }
    }
}
