//! This crate defines properties and traits for models.

pub mod data;
pub mod geometry_model;
pub mod geometry_object;
pub mod model;
pub mod neutral;

type Configuration = nalgebra::DVector<f64>;
