use dynamics_spatial::{
    py_configuration::{PyConfiguration, PyConfigurationInput},
    py_motion::PySpatialMotion,
    py_se3::PySE3,
};
use pyo3::{exceptions::PyValueError, prelude::*};

use crate::{
    joint_data::{JointData, JointDataWrapper},
    py_joint::PyJointWrapper,
};

/// Mutable properties of the joints.
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
        PyConfiguration(self.inner.get_joint_q().clone())
    }

    #[getter]
    #[must_use]
    /// Returns the joint velocity vector.
    pub fn joint_v(&self) -> PyConfiguration {
        PyConfiguration(self.inner.get_joint_v().clone())
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
    /// Updates the joint data with the current position and velocity configurations.
    pub fn update(
        &mut self,
        joint_model: &PyJointWrapper,
        joint_q: PyConfigurationInput,
        joint_v: Option<PyConfigurationInput>,
    ) -> PyResult<()> {
        let joint_q = joint_q.to_configuration(joint_model.nq())?;
        let joint_v = joint_v
            .map(|v| v.to_configuration(joint_model.nv()))
            .transpose()?;

        match self
            .inner
            .update(&joint_model.inner, &joint_q, joint_v.as_ref())
        {
            Ok(()) => Ok(()),
            Err(e) => Err(PyValueError::new_err(format!(
                "Failed to update joint data: {e:?}"
            ))),
        }
    }

    #[getter]
    /// Returns the joint velocity as a spatial motion.
    pub fn get_joint_velocity(&self) -> PySpatialMotion {
        PySpatialMotion {
            inner: self.inner.get_joint_velocity().clone(),
        }
    }

    #[getter]
    /// Returns the joint velocity as a spatial motion.
    pub fn get_v(&self) -> PySpatialMotion {
        self.get_joint_velocity()
    }
}
