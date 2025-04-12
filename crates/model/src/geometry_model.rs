//! Model for a geometry structure, containing multiple geometry objects.

use crate::geometry_object::{GeometryObject, PyGeometryObject};
use pyo3::prelude::*;

/// A model for a geometry structure, containing multiple geometry objects.
pub struct GeometryModel {
    /// The list of geometry objects contained in this model.
    pub models: Vec<GeometryObject>,
}

impl GeometryModel {
    /// Creates a new `GeometryModel` with an empty list of objects.
    pub fn new() -> Self {
        GeometryModel { models: Vec::new() }
    }

    /// Adds a new geometry model to the list of models.
    ///
    /// # Arguments
    ///
    /// * `object` - The geometry object to be added to the model.
    pub fn add_geometry_object(&mut self, object: GeometryObject) {
        self.models.push(object);
    }
}

/// A Python wrapper for the `GeometryModel` struct.
#[pyclass(name = "GeometryModel")]
pub struct PyGeometryModel {
    pub inner: GeometryModel,
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
}
