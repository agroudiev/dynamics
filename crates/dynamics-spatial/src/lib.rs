//! This crate is part of the `dynamics` ecosystem, and is not intended for direct use.
//!
//! ## Overview
//! This crate provides spatial algebra utilities for rigid body dynamics.
//! It implements 6-dimensional vectors, encoding both rotational and translational components,
//! as well as spatial transformations and operations.
//!
//! ## Spatial algebra
//! Most computations of the main algorithms of `dynamics` (e.g. forward kinematics, inverse dynamics) are performed using spatial algebra.
//! Spatial vectors are 6-dimensional vectors that encode both rotational and translational components of motion or force,
//! making both equations and implementations more compact and efficient.
//!
//! ## Core types
//! The core types provided by this crate are:
//! - [`motion::SpatialMotion`]: represents spatial motion vectors (angular and linear velocity).
//! - [`force::SpatialForce`]: represents spatial force vectors (torque and force).
//! - [`se3::SE3`]: represents spatial transformations (rotation and translation), that is elements of the $\text{SE}(3)$ group.
//! - [`so3::SO3`]: represents 3D rotations, that is elements of the $\text{SO}(3)$ group.
//! - [`symmetric3::Symmetric3`]: represents symmetric 3x3 matrices, used for inertia tensors.
//!
//! The following types are also provided for convenience:
//! - [`vector3d::Vector3D`]: represents 3D vectors, used for underlying representations of motion and force vectors.
//! - [`vector6d::Vector6D`]: represents 6D vectors, used for underlying representations of spatial motion and force vectors.   

pub mod color;
pub mod configuration;
pub mod force;
pub mod inertia;
pub mod jacobian;
pub mod motion;
pub mod se3;
pub mod so3;
pub mod symmetric3;
pub mod vector3d;
pub mod vector6d;

#[cfg(feature = "python")]
pub mod py_configuration;
#[cfg(feature = "python")]
pub mod py_force;
#[cfg(feature = "python")]
pub mod py_jacobian;
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
