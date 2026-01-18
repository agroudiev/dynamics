//! Model for a geometry structure, containing multiple geometry objects.

use crate::{
    data::{Data, GeometryData, PyData, PyGeometryData},
    geometry_object::{GeometryObject, PyGeometryObject},
};
use pyo3::prelude::*;

/// A model for a geometry structure, containing multiple geometry objects.
#[derive(Clone, Debug)]
pub struct GeometryModel {
    /// The list of geometry objects contained in this model.
    pub objects: Vec<GeometryObject>,
}

impl Default for GeometryModel {
    fn default() -> Self {
        GeometryModel::new()
    }
}

impl GeometryModel {
    /// Creates a new [`GeometryModel`] with an empty list of objects.
    #[must_use]
    pub fn new() -> Self {
        GeometryModel {
            objects: Vec::new(),
        }
    }

    /// Adds a new geometry model to the list of models.
    ///
    /// # Arguments
    ///
    /// * `object` - The geometry object to be added to the model.
    pub fn add_geometry_object(&mut self, object: GeometryObject) -> usize {
        self.objects.push(object);
        self.objects.len() - 1
    }

    #[must_use]
    pub fn get_geometry_id(&self, name: &str) -> Option<usize> {
        self.objects.iter().position(|o| o.name == name)
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
    #[must_use]
    pub fn create_data(&self, data: &Data) -> GeometryData {
        let mut geom_data = GeometryData::default();
        geom_data.update_geometry_data(data, self);
        geom_data
    }
}

/// A Python wrapper for the [`GeometryModel`] struct.
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
    /// Creates a new [`GeometryModel`] with an empty list of objects.
    #[new]
    #[must_use]
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
    ///
    /// # Returns
    ///
    /// The index of the added geometry object.
    pub fn add_geometry_object(&mut self, object: &PyGeometryObject) -> usize {
        self.inner.add_geometry_object(object.inner.clone())
    }

    /// Returns the list of geometry objects in the model.
    #[getter]
    #[must_use]
    pub fn geometry_objects(&self) -> Vec<PyGeometryObject> {
        self.inner
            .objects
            .iter()
            .map(|obj| PyGeometryObject { inner: obj.clone() })
            .collect()
    }

    /// Creates a new `GeometryData` object based on the provided model and data.
    ///
    /// # Arguments
    ///
    /// * `data` - The data to be used for creating the geometry data.
    /// # Returns
    ///
    /// A `GeometryData` object containing the geometry data for the model.
    pub fn create_data(&self, data: &PyData) -> PyResult<PyGeometryData> {
        Ok(PyGeometryData {
            inner: self.inner.create_data(&data.inner),
        })
    }

    fn __repr__(slf: PyRef<'_, Self>) -> String {
        format!("{:#?}", slf.inner)
    }

    #[getter]
    #[must_use]
    pub fn ngeoms(&self) -> usize {
        self.inner.objects.len()
    }
}
