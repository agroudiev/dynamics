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
//!
//! ## Joint types
//! The following joint types are implemented:
//! - [`continuous::JointModelContinuous`]: Represents a continuous joint that can rotate indefinitely around a fixed axis.
//! - [`prismatic::JointModelPrismatic`]: Represents a prismatic joint that allows linear motion along a fixed axis.
//! - [`revolute::JointModelRevolute`]: Represents a revolute joint that allows rotation around a fixed axis with limits. This is similar to the continuous joint but with defined limits on the rotation.
//! - [`fixed::JointModelFixed`]: Represents a fixed joint with no degrees of freedom.
//!
//! The following table summarizes the number of configuration variables (`nq`) and velocity variables (`nv`) for each joint type:
//! <center>
//!
//! | Joint      | `nq` | `nv` | Description        |
//! |------------|------|------|--------------------|
//! | Continuous | 2    | 1    | Unbounded rotation |
//! | Prismatic  | 1    | 1    | Linear motion      |
//! | Revolute   | 1    | 1    | Bounded rotation   |
//! | Fixed      | 0    | 0    | No motion          |
//!
//! </center>

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
