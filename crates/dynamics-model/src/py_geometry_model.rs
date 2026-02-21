use pyo3::prelude::*;

use crate::{
    geometry_model::GeometryModel,
    py_data::{PyData, PyGeometryData},
    py_geometry_object::PyGeometryObject,
};

/// A model for a geometry structure, containing multiple geometry objects.
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
    /// Creates a new GeometryModel with an empty list of objects.
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
    /// Number of geometry objects in the model.
    pub fn ngeoms(&self) -> usize {
        self.inner.objects.len()
    }

    /// Returns the ID of the geometry object with the given name, if it exists.
    pub fn get_geometry_id(&self, name: &str) -> Option<usize> {
        self.inner.get_geometry_id(name)
    }
}
