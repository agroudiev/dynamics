//! Model for a geometry structure, containing multiple geometry objects.

use crate::{
    data::{Data, GeometryData},
    geometry_object::GeometryObject,
};

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
