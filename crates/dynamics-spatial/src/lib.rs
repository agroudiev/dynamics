//! This crate is part of the `dynamics` ecosystem, and is not intended for direct use.
//!
//! This crate provides spatial algebra utilities for rigid body dynamics.
//! It implements 6-dimensional vectors, encoding both rotational and translational components,
//! as well as spatial transformations and operations.

pub mod color;
pub mod configuration;
pub mod force;
pub mod inertia;
pub mod motion;
pub mod se3;
pub mod so3;
pub mod symmetric3;
pub mod transform;
pub mod vector3d;
pub mod vector6d;

#[cfg(feature = "python")]
pub mod py_configuration;
#[cfg(feature = "python")]
pub mod py_force;
#[cfg(feature = "python")]
pub mod py_motion;
#[cfg(feature = "python")]
pub mod py_se3;
#[cfg(feature = "python")]
pub mod py_symmetric3;
#[cfg(feature = "python")]
pub mod py_vector3d;
#[cfg(feature = "python")]
pub mod py_vector6d;
