//! Model for a geometry structure, containing multiple geometry objects.

use std::collections::HashMap;

use crate::{
    data::{Data, GeometryData},
    geometry_object::{GeometryObject, PyGeometryObject},
};
use pyo3::prelude::*;

/// A model for a geometry structure, containing multiple geometry objects.
pub struct GeometryModel {
    /// The list of geometry objects contained in this model.
    pub models: HashMap<usize, GeometryObject>,
    /// The indices of the geometry objects by name.
    pub indices: HashMap<String, usize>,
}

impl Default for GeometryModel {
    fn default() -> Self {
        GeometryModel::new()
    }
}

impl GeometryModel {
    /// Creates a new `GeometryModel` with an empty list of objects.
    pub fn new() -> Self {
        GeometryModel {
            models: HashMap::new(),
            indices: HashMap::new(),
        }
    }

    /// Adds a new geometry model to the list of models.
    ///
    /// # Arguments
    ///
    /// * `object` - The geometry object to be added to the model.
    pub fn add_geometry_object(&mut self, mut object: GeometryObject) -> usize {
        let id = self.models.len();
        object.id = id;
        self.models.insert(id, object);
        self.indices
            .insert(self.models.get(&id).unwrap().name.clone(), id);
        id
    }

    /// Creates a new `GeometryData` object based on the provided model and data.
    ///
    /// # Arguments
    ///
    /// * `model` - The model to be used for creating the geometry data.
    /// * `data` - The data to be used for creating the geometry data.
    ///
    /// # Returns
    ///
    /// A `GeometryData` object containing the geometry data for the model.
    pub fn create_data(&self, data: &Data) -> GeometryData {
        let mut geom_data = GeometryData::default();
        geom_data.update_geometry_data(data, self);
        geom_data
    }
}

/// A Python wrapper for the `GeometryModel` struct.
#[pyclass(name = "GeometryModel")]
pub struct PyGeometryModel {
    pub inner: GeometryModel,
}

impl Default for PyGeometryModel {
    fn default() -> Self {
        PyGeometryModel::new()
    }
}

#[pymethods]
impl PyGeometryModel {
    /// Creates a new `GeometryModel` with an empty list of objects.
    #[new]
    pub fn new() -> Self {
        PyGeometryModel {
            inner: GeometryModel::new(),
        }
    }

    /// Adds a new geometry model to the list of models.
    ///
    /// # Arguments
    ///
    /// * `object` - The geometry object to be added to the model.
    pub fn add_geometry_object(&mut self, object: &PyGeometryObject) {
        self.inner.add_geometry_object(object.inner.clone());
    }

    /// Returns the list of geometry objects in the model.
    #[getter]
    pub fn geometry_objects(&self) -> Vec<PyGeometryObject> {
        self.inner
            .models
            .values()
            .map(|obj| PyGeometryObject { inner: obj.clone() })
            .collect()
    }

    /// Adds a new geometry object to the list of objects in the model.
    pub fn add_geometry_object_from_py(&mut self, object: &PyGeometryObject) {
        self.inner.add_geometry_object(object.inner.clone());
    }
}
