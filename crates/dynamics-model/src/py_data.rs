use crate::{
    data::{Data, GeometryData},
    py_geometry_model::PyGeometryModel,
    py_model::PyModel,
};
use dynamics_joint::py_joint_data::PyJointDataWrapper;
use dynamics_spatial::py_se3::PySE3;
use pyo3::prelude::*;

/// A Python wrapper for the `Data` struct.
#[pyclass(name = "Data")]
pub struct PyData {
    pub inner: Data,
}

#[pymethods]
impl PyData {
    #[new]
    /// Creates a new `Data` object.
    ///
    /// # Arguments
    ///
    /// * `model` - The model object.
    ///
    /// # Returns
    /// A new `Data` object corresponding to the given model.
    #[must_use]
    pub fn new(model: &PyModel) -> Self {
        PyData {
            inner: model.inner.create_data(),
        }
    }

    #[getter]
    /// Returns the placements of the joints in the world frame.
    #[must_use]
    pub fn joint_placements(&self) -> Vec<PySE3> {
        self.inner
            .joint_placements
            .iter()
            .map(|p| PySE3 { inner: *p })
            .collect()
    }

    #[getter]
    /// Returns the placements of the frames in the world frame.
    #[must_use]
    pub fn frame_placements(&self) -> Vec<PySE3> {
        self.inner
            .frame_placements
            .iter()
            .map(|p| PySE3 { inner: *p })
            .collect()
    }

    #[getter]
    #[allow(non_snake_case)]
    /// Returns the placements of the joints in the world frame.
    ///
    /// This is an alias for `joint_placements` to match the Pinocchio API.
    #[must_use]
    pub fn oMi(&self) -> Vec<PySE3> {
        self.joint_placements()
    }

    #[getter]
    #[allow(non_snake_case)]
    /// Returns the placements of the frames in the world frame.
    ///
    /// This is an alias for `frame_placements` to match the Pinocchio API.
    #[must_use]
    pub fn oMf(&self) -> Vec<PySE3> {
        self.frame_placements()
    }

    #[getter]
    pub fn joints(&self) -> Vec<PyJointDataWrapper> {
        self.inner
            .joint_data
            .iter()
            .map(|jd| PyJointDataWrapper {
                inner: jd.clone_box(),
            })
            .collect()
    }
}

/// A Python wrapper for the `GeometryData` struct.
#[pyclass(name = "GeometryData")]
pub struct PyGeometryData {
    pub inner: GeometryData,
}

#[pymethods]
impl PyGeometryData {
    #[new]
    /// Creates a new `GeometryData` object.
    ///
    /// # Arguments
    ///
    /// * `model` - The model object.
    /// * `data` - The data object.
    /// * `geom_model` - The geometry model object.
    ///
    /// # Returns
    /// A new `GeometryData` object.
    #[must_use]
    pub fn new(data: &PyData, geom_model: &PyGeometryModel) -> Self {
        let mut geom_data = GeometryData::default();
        geom_data.update_geometry_data(&data.inner, &geom_model.inner);
        PyGeometryData { inner: geom_data }
    }

    /// Returns the placement of the object of given index in the world frame.
    ///
    /// # Arguments
    ///
    /// * `object_index` - The index of the object.
    ///
    /// # Returns
    /// The object placement if it exists, otherwise `None`.
    pub fn get_object_placement(&self, object_index: usize) -> PyResult<PySE3> {
        match self.inner.get_object_placement(object_index) {
            Some(placement) => Ok(PySE3 { inner: *placement }),
            None => Err(pyo3::exceptions::PyKeyError::new_err(format!(
                "Object with index {object_index} not found"
            ))),
        }
    }

    /// Updates the geometry data using the updated model data and geometry model.
    ///
    /// # Arguments
    ///
    /// * `data` - The model data object.
    /// * `geom_model` - The geometry model object.
    pub fn update_geometry_data(&mut self, data: &PyData, geom_model: &PyGeometryModel) {
        self.inner
            .update_geometry_data(&data.inner, &geom_model.inner);
    }
}
