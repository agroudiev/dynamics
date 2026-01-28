//! This crate is part of the `dynamics` ecosystem, and is not intended for direct use.
//!
//! This module provides structures and traits to represent joints in a robot model.

pub mod joint;
pub mod joint_data;
pub mod limits;

pub mod continuous;
pub mod fixed;
pub mod prismatic;
pub mod revolute;

#[cfg(feature = "python")]
pub mod py_joint;
#[cfg(feature = "python")]
pub mod py_joint_data;
