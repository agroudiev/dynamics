//! # **`dynamics`**: A Rust library for Rigid Body Dynamics
//!
//! `dynamics` is a Rust library for rigid body dynamics computations, allowing
//! users to model and simulate the motion of articulated rigid body systems such
//! as robots. It provides efficient algorithms for computing kinematics, dynamics,
//! and other related quantities.
//!
//! On top of Rust API, Python bindings are provided for easy integration into Python projects.
//! The Python package [`rdynamics-py`](https://pypi.org/project/rdynamics-py/) can be installed via `pip`.
//!
//! This library has two inspiration sources:
//! - Roy Featherstone's book ["Rigid Body Dynamics Algorithms"](https://link.springer.com/book/10.1007/978-1-4899-7560-7), which provides the theoretical
//!   foundation for many of the algorithms implemented in this library.
//! - The C++ library [Pinocchio](https://github.com/stack-of-tasks/pinocchio/), a widely-used
//!   library for rigid body dynamics in robotics, which served as a reference for the API design
//!   and implementation details.
//!
//! ## Features
//! - Representation of rigid body systems using articulated body models.
//! - Parsing of model files in the folowing formats:
//!   * URDF (Unified Robot Description Format)
//!   * *SDF (Simulation Description Format)* (WIP)
//! - The following algorithms:
//!   * Forward kinematics
//!   * *Jacobians computation* (WIP)
//!   * *Inverse dynamics (RNEA)* (WIP)
//!   * *Forward dynamics (ABA)* (WIP)
//!
//! ## Crates
//! The `dynamics` library is organized into several crates, each focusing on a specific aspect
//! of rigid body dynamics:
//! - [`dynamics-rs`](https://docs.rs/crate/dynamics-rs/latest): the main crate that provides high-level functionalities and interfaces.
//! - [`dynamics-spatial`](https://docs.rs/crate/dynamics-spatial/latest): implements spatial algebra used in rigid body dynamics, such as spatial vectors and transformations.
//! - [`dynamics-model`](https://docs.rs/crate/dynamics-model/latest): contains model and data structures for rigid body systems.
//! - [`dynamics-joint`](https://docs.rs/crate/dynamics-joint/latest): implements various joint types and their properties.
//! - [`dynamics-inertia`](https://docs.rs/crate/dynamics-inertia/latest): provides inertia-related computations and data structures.
//! - [`dynamics-parse`](https://docs.rs/crate/dynamics-parse/latest): utilities for parsing model files and configurations.
//!
//! For collision checking functionalities, the [`collider-rs`](https://docs.rs/crate/collider-rs/latest) crate is used.

pub use collider_rs as collider;
pub use dynamics_inertia as inertia;
pub use dynamics_joint as joint;
pub use dynamics_model as model;
pub use dynamics_parse as parse;
pub use dynamics_spatial as spatial;
