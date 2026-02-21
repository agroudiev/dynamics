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

/// Generates a random configuration for the given model.
#[pyfunction(name = "random_configuration")]
pub fn py_random_configuration(model: &mut PyModel) -> PyConfiguration {
    let q = random_configuration(&model.inner);
    PyConfiguration(q)
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
    /// Name of the model.
    fn name(&self) -> &str {
        &self.inner.name
    }

    #[setter]
    /// Sets the name of the model.
    fn set_name(&mut self, name: String) {
        self.inner.name = name;
    }

    #[getter]
    /// Number of configuration variables in the model.
    fn nq(&self) -> usize {
        self.inner.nq
    }

    #[getter]
    /// Number of velocity variables in the model.
    fn nv(&self) -> usize {
        self.inner.nv
    }

    #[getter]
    /// Gravity vector of the model.
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

    /// Appends a body of given inertia to the joint with given id.
    ///
    /// # Arguments
    ///
    /// * `joint_id` - The identifier of the joint to append the body to.
    /// * `inertia` - The inertia of the body to append.
    /// * `placement` - The placement of the body in the joint frame.
    ///
    /// # Returns
    ///
    /// A result indicating success or failure.
    fn append_body_to_joint(
        &mut self,
        joint_id: usize,
        inertia: &PyInertia,
        placement: &PySE3,
    ) -> PyResult<()> {
        match self
            .inner
            .append_body_to_joint(joint_id, &inertia.inner, placement.inner)
        {
            Ok(_) => Ok(()),
            Err(model_error) => Err(PyValueError::new_err(format!("{model_error:?}"))),
        }
    }

    #[getter]
    /// Number of joints in the model.
    fn njoints(&self) -> usize {
        self.inner.njoints()
    }

    #[getter]
    /// Number of frames in the model.
    fn nframes(&self) -> usize {
        self.inner.nframes()
    }

    #[pyo3(signature = (name))]
    /// Returns the index of the joint with the given name.
    fn get_joint_id(&self, name: &str) -> Option<usize> {
        self.inner.get_joint_id(name)
    }

    /// Creates a new [`Data`] structure for this model.
    fn create_data(&self) -> PyData {
        let data = self.inner.create_data();
        PyData { inner: data }
    }

    #[getter]
    /// Names of the joints.
    fn get_joint_names(&self) -> &[String] {
        &self.inner.joint_names
    }

    #[getter]
    /// Parent joint indices for each joint. The root joint is its own parent.
    fn get_joint_parents(&self) -> &[usize] {
        &self.inner.joint_parents
    }

    #[getter]
    /// Placements of the joints in the parent frame.
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
    /// Joint models.
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
    /// Inertias of the bodies at each joint.
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
    /// Operational frames at each joint
    fn get_frames(&self) -> Vec<PyFrame> {
        self.inner
            .frames
            .iter()
            .map(|frame| PyFrame {
                inner: frame.clone(),
            })
            .collect()
    }

    /// Adds a frame (fixed joint) to the model.
    ///
    /// # Arguments
    ///
    /// * `placement` - The placement of the frame in the parent frame.
    /// * `name` - The name of the frame.
    ///
    /// # Returns
    ///
    /// The identifier of the frame.
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
    /// Returns the index of the frame with the given name.
    pub fn get_frame_id(&self, name: &str, frame_type: Option<FrameType>) -> Option<usize> {
        self.inner.get_frame_id(name, frame_type.as_ref())
    }

    /// Prints the joint tree of the model.
    pub fn print_joint_tree(&self) -> PyResult<()> {
        match self.inner.print_joint_tree() {
            Ok(()) => Ok(()),
            Err(io_error) => Err(PyValueError::new_err(format!("{io_error:?}"))),
        }
    }
}
