//! Structures and traits for data representation.
//! The `Model` object represents the properties of the robot model which are immutable and are not changed or computed by the algorithms.
//! The `Data` object represents the properties of the robot model which are obtained by the algorithms, such as the joint positions, velocities, accelerations.

use joint::data::JointDataWrapper;
use nalgebra::IsometryMatrix3;
use pyo3::{PyResult, pyclass, pymethods};
use spatial::se3::PySE3;
use std::collections::HashMap;

use crate::{
    geometry_model::{GeometryModel, PyGeometryModel},
    model::{Model, PyModel, WORLD_FRAME_ID},
};

#[derive(Default)]
pub struct Data {
    /// The data of the joints
    joints_data: HashMap<usize, JointDataWrapper>,
    /// The placements of the joints in the world frame
    joints_placements: HashMap<usize, IsometryMatrix3<f64>>,
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
    pub fn new(
        joints_data: HashMap<usize, JointDataWrapper>,
        joints_placements: HashMap<usize, IsometryMatrix3<f64>>,
    ) -> Self {
        Self {
            joints_data,
            joints_placements,
        }
    }

    /// Returns the joint data for a given joint index.
    ///
    /// # Arguments
    ///
    /// * `joint_index` - The index of the joint.
    ///
    /// # Returns
    /// An `Option` containing the joint data if it exists, otherwise `None`.
    pub fn get_joint_data(&self, joint_index: usize) -> Option<&JointDataWrapper> {
        self.joints_data.get(&joint_index)
    }

    /// Returns the joint placement for a given joint index.
    ///
    /// # Arguments
    ///
    /// * `joint_index` - The index of the joint.
    ///
    /// # Returns
    /// An `Option` containing the joint placement if it exists, otherwise `None`.
    pub fn get_joint_placement(&self, joint_index: usize) -> Option<&IsometryMatrix3<f64>> {
        self.joints_placements.get(&joint_index)
    }
}

#[pyclass(name = "Data")]
/// A Python wrapper for the `Data` struct.
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
}

#[derive(Default)]
pub struct GeometryData {
    /// The placements of the objects in the world frame
    object_placements: HashMap<usize, IsometryMatrix3<f64>>,
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
    pub fn get_object_placement(&self, object_index: usize) -> Option<&IsometryMatrix3<f64>> {
        self.object_placements.get(&object_index)
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
    ///
    /// As this function uses the joint placements from the model data (`data`), it should be called after the model data is updated.
    pub fn update_geometry_data(&mut self, model: &Model, data: &Data, geom_model: &GeometryModel) {
        self.object_placements.clear();

        for (object_id, object) in geom_model.models.iter() {
            if object.parent_joint == WORLD_FRAME_ID {
                let parent_frame_id = object.parent_frame;
                let parent_frame_placement = model.frames.get(&parent_frame_id).unwrap();
                let object_placement = parent_frame_placement * object.placement;
                self.object_placements.insert(*object_id, object_placement);
            } else {
                let parent_joint_id = object.parent_joint;
                let parent_joint_placement = data.get_joint_placement(parent_joint_id).unwrap();
                let object_placement = parent_joint_placement * object.placement;
                self.object_placements.insert(*object_id, object_placement);
            }
        }
    }
}

#[pyclass(name = "GeometryData")]
/// A Python wrapper for the `GeometryData` struct.
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
    pub fn new(model: &PyModel, data: &PyData, geom_model: &PyGeometryModel) -> Self {
        let mut geom_data = GeometryData::default();
        geom_data.update_geometry_data(&model.inner, &data.inner, &geom_model.inner);
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
}
