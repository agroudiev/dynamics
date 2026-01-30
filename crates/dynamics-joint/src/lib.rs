//! This crate is part of the `dynamics` ecosystem, and is not intended for direct use.
//!
//! This module provides structures and traits to represent joints in a robot model.
//!
//! ## Architecture
//! In order to use static dispatch for different joint types while maintaining a common interface, the following architecture is implemented:
//! * The `JointModel` trait defines the common interface, implemented by specific joint types.
//! * Different joint types (e.g., `JointModelContinuous`, `JointModelPrismatic`, `JointModelRevolute`, `JointModelFixed`) implement the `JointModel` trait.
//! * The `JointModelImpl` enum encapsulates different joint model implementations.
//! * The `JointWrapper` struct provides a unified interface to interact with different joint types through the `JointModel` trait. This is the main entry point for users of the library.

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
