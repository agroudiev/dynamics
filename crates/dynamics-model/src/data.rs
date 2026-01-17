//! `Data` structure containing the mutable properties of the robot.

use dynamics_joint::data::JointDataWrapper;
use dynamics_spatial::se3::{PySE3, SE3};
use pyo3::{PyResult, pyclass, pymethods};

use crate::{
    geometry_model::{GeometryModel, PyGeometryModel},
    model::PyModel,
};

/// Structure containing the mutable properties of the robot.
#[derive(Default)]
pub struct Data {
    /// The data of the joints
    pub joint_data: Vec<JointDataWrapper>,
    /// The placements of the joints in the world frame (oMi)
    pub joint_placements: Vec<SE3>,
}

impl Data {
    /// Creates a new `Data` object.
    ///
    /// # Arguments
    ///
    /// * `joints_data` - A HashMap of joint indices to their data.
    /// * `joints_placements` - A HashMap of joint indices to their placements.
    ///
    /// # Returns
    /// A new `Data` object.
    pub fn new(joint_data: Vec<JointDataWrapper>, joint_placements: Vec<SE3>) -> Self {
        Self {
            joint_data,
            joint_placements,
        }
    }
}

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
    pub fn new(model: &PyModel) -> Self {
        PyData {
            inner: model.inner.create_data(),
        }
    }

    #[getter]
    /// Returns the placements of the joints in the world frame.
    pub fn joint_placements(&self) -> Vec<PySE3> {
        self.inner
            .joint_placements
            .iter()
            .map(|p| PySE3 { inner: *p })
            .collect()
    }

    #[getter]
    #[allow(non_snake_case)]
    /// Returns the placements of the joints in the world frame.
    ///
    /// This is an alias for `joint_placements` to match the Pinocchio API.
    pub fn oMi(&self) -> Vec<PySE3> {
        self.joint_placements()
    }
}

/// Structure containing the mutable geometric data of the models.
#[derive(Default)]
pub struct GeometryData {
    /// The placements of the objects in the world frame
    pub object_placements: Vec<SE3>,
}

impl GeometryData {
    /// Returns the placement of the object of given index in the world frame.
    ///
    /// # Arguments
    ///
    /// * `object_index` - The index of the object.
    ///
    /// # Returns
    /// An `Option` containing the object placement if it exists, otherwise `None`.
    pub fn get_object_placement(&self, object_index: usize) -> Option<&SE3> {
        self.object_placements.get(object_index)
    }

    /// Updates the geometry data with the given model and geometry model.
    ///
    /// # Arguments
    ///
    /// * `model` - The model containing the updated joint placements.
    /// * `data` - The data containing the joint placements.
    /// * `geom_model` - The geometry model containing the object placements.
    ///
    /// # Note
    /// As this function uses the joint placements from the model data (`data`), it should be called after the model data is updated.
    pub fn update_geometry_data(&mut self, data: &Data, geom_model: &GeometryModel) {
        self.object_placements.clear();

        for object in geom_model.objects.iter() {
            let parent_joint_id = object.parent_joint;
            let parent_joint_placement = data.joint_placements[parent_joint_id];
            let object_placement = parent_joint_placement * object.placement;
            self.object_placements.push(object_placement);
        }
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
                "Object with index {} not found",
                object_index
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
