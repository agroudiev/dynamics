//! This crate is part of the `dynamics` ecosystem, and is not intended for direct use.
//!
//! ## Overview
//! This crate provides parsers for various robot description formats.
//! The core feature is to generate a [`dynamics_model::model::Model`] from a robot description file, which can then be used for dynamics computations.
//!
//! ## Supported formats
//! Currently, the only supported format is URDF, but support for other formats (e.g. SDF) may be added in the future.

pub mod errors;
pub mod urdf;

#[cfg(feature = "python")]
pub mod py_urdf;

#[cfg(test)]
mod tests;
