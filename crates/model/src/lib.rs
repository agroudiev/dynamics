//! This crate defines properties and traits for models, data, and geometry objects.
//! All these structures represent the robot model and its properties.
//! 
//! The `Model` object represents the properties of the robot model which are immutable and are not changed or computed by the algorithms.
//! Such properties are only altered by the user.
//! 
//! The `Data` object represents the properties of the robot model which are obtained by the algorithms.
//! They are not directly provided by the user and might change during the execution of the algorithms.
//! 
//! For instance, the `Model` object contains the joint placements with respect to the parent frame,
//! while the `Data` object contains the joint placements with respect to the world frame.
//! The local placement of objects should not change without user intervention, while the world placement of objects might change when computing the forward kinematics.

pub mod data;
pub mod geometry_model;
pub mod geometry_object;
pub mod model;
pub mod neutral;

type Configuration = nalgebra::DVector<f64>;
