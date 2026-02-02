use dynamics_inertia::py_inertia::PyInertia;
use dynamics_joint::py_joint::PyJointWrapper;
use dynamics_spatial::{py_configuration::PyConfiguration, py_se3::PySE3};
use numpy::ToPyArray;
use numpy::ndarray::Array1;
use pyo3::{exceptions::PyValueError, prelude::*};

use crate::{
    frame::FrameType,
    model::{Model, random_configuration},
    py_data::PyData,
    py_frame::PyFrame,
};

#[pyfunction(name = "random_configuration")]
pub fn py_random_configuration(model: &mut PyModel) -> PyConfiguration {
    let q = random_configuration(&model.inner);
    PyConfiguration::new(q)
}

/// A [`Model`] is a data structure that contains the information about the robot model,
/// including the joints models, placements, the link inertias, and the frames.
#[pyclass(name = "Model")]
pub struct PyModel {
    pub inner: Model,
}

#[pymethods]
impl PyModel {
    /// Creates a new empty [`Model`].
    // TODO: update this function for more flexibility
    #[new]
    fn new_empty() -> Self {
        Self {
            inner: Model::new_empty(),
        }
    }

    #[getter]
    fn name(&self) -> &str {
        &self.inner.name
    }

    #[setter]
    fn set_name(&mut self, name: String) {
        self.inner.name = name;
    }

    #[getter]
    fn nq(&self) -> usize {
        self.inner.nq
    }

    #[getter]
    fn nv(&self) -> usize {
        self.inner.nv
    }

    #[getter]
    fn gravity(&self, py: Python) -> Py<PyAny> {
        Array1::from_shape_vec([3], self.inner.gravity.as_slice().to_vec())
            .unwrap()
            .to_pyarray(py)
            .into_any()
            .unbind()
    }

    /// Adds a joint to the model.
    ///
    /// # Arguments
    ///
    /// * `parent_id` - The identifier of the parent joint.
    /// * `joint_model` - The joint model to add.
    /// * `placement` - The placement of the joint in the parent frame.
    /// * `name` - The name of the joint.
    #[pyo3(signature = (parent_id, joint_model, placement, name))]
    fn add_joint(
        &mut self,
        parent_id: usize,
        joint_model: &PyJointWrapper,
        placement: &PySE3,
        name: String,
    ) -> PyResult<usize> {
        match self
            .inner
            .add_joint(parent_id, joint_model.inner.clone(), placement.inner, name)
        {
            Ok(id) => Ok(id),
            Err(model_error) => Err(PyValueError::new_err(format!("{model_error:?}"))),
        }
    }

    // fn append_body_to_joint(
    //     &mut self,
    //     joint_id: usize,
    //     inertia: &PyInertia,
    //     placement: &PySE3,
    // ) -> PyResult<()> {
    //     match self
    //         .inner
    //         .append_body_to_joint(joint_id, inertia.inner.clone(), placement.inner)
    //     {
    //         Ok(_) => Ok(()),
    //         Err(model_error) => Err(PyValueError::new_err(format!("{:?}", model_error))),
    //     }
    // }

    #[getter]
    fn njoints(&self) -> usize {
        self.inner.njoints()
    }

    #[getter]
    fn nframes(&self) -> usize {
        self.inner.nframes()
    }

    #[pyo3(signature = (name))]
    fn get_joint_id(&self, name: &str) -> Option<usize> {
        self.inner.get_joint_id(name)
    }

    #[pyo3(signature = ())]
    fn create_data(&self) -> PyData {
        let data = self.inner.create_data();
        PyData { inner: data }
    }

    #[getter]
    fn get_joint_names(&self) -> &[String] {
        &self.inner.joint_names
    }

    #[getter]
    fn get_joint_parents(&self) -> &[usize] {
        &self.inner.joint_parents
    }

    #[getter]
    fn get_joint_placements(&self) -> Vec<Py<PySE3>> {
        Python::with_gil(|py| {
            self.inner
                .joint_placements
                .iter()
                .map(|placement| Py::new(py, PySE3 { inner: *placement }).unwrap())
                .collect()
        })
    }

    #[getter]
    fn get_joint_models(&self) -> Vec<PyJointWrapper> {
        self.inner
            .joint_models
            .iter()
            .map(|joint_model| PyJointWrapper {
                inner: joint_model.clone(),
            })
            .collect()
    }

    #[getter]
    fn get_inertias(&self) -> Vec<PyInertia> {
        self.inner
            .inertias
            .iter()
            .map(|inertia| PyInertia {
                inner: inertia.clone(),
            })
            .collect()
    }

    #[getter]
    fn get_frames(&self) -> Vec<PyFrame> {
        self.inner
            .frames
            .iter()
            .map(|frame| PyFrame {
                inner: frame.clone(),
            })
            .collect()
    }

    fn add_frame(&mut self, frame: PyFrame, append_inertia: bool) -> PyResult<usize> {
        match self.inner.add_frame(frame.inner, append_inertia) {
            Ok(id) => Ok(id),
            Err(model_error) => Err(PyValueError::new_err(format!("{model_error:?}"))),
        }
    }

    fn __repr__(slf: PyRef<'_, Self>) -> String {
        format!("{:#?}", slf.inner)
    }

    #[pyo3(signature = (name, frame_type=None))]
    pub fn get_frame_id(&self, name: &str, frame_type: Option<FrameType>) -> Option<usize> {
        self.inner.get_frame_id(name, frame_type)
    }
}
