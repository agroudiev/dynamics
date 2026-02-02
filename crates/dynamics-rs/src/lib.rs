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
//! ## Example
//! The following example demonstrates how to load a robot model from a URDF file,
//! compute its forward kinematics for a random configuration, and print the placement
//! of a specific joint and frame.
//!
//! ```no_run
//! use dynamics_model::forward_kinematics::update_frame_placements;
//! use dynamics_model::model::random_configuration;
//! use dynamics_rs::model::forward_kinematics::forward_kinematics;
//! use dynamics_rs::parse::urdf::build_models_from_urdf;
//!
//! fn main() {
//!     let urdf_path = "./examples/descriptions/ur5/ur5_robot.urdf"; // Path to the URDF file
//!     let mesh_path = "./examples/descriptions/ur5/"; // Optional mesh path
//!
//!     // Build models from the URDF file
//!     let (model, _coll_model, _viz_model) =
//!         build_models_from_urdf(urdf_path, Some(mesh_path)).expect("Failed to parse URDF file");
//!
//!     // Generate a random configuration
//!     let q = random_configuration(&model);
//!     println!("Random configuration q: {}", q);
//!
//!     // Create data structure for the model
//!     let mut data = model.create_data();
//!
//!     // Compute forward kinematics
//!     forward_kinematics(&model, &mut data, &q, &None, &None)
//!         .expect("Failed to compute forward kinematics");
//!
//!     // Print the placement of the joint 'wrist_3_joint'
//!     let id = model.get_joint_id("wrist_3_joint").unwrap();
//!     let placement = &data.joint_placements[id];
//!     println!("Placement of 'wrist_3_joint':\n{:?}", placement);
//!
//!     // Compute the frame placements
//!     update_frame_placements(&model, &mut data);
//!
//!     // Print the placement of the frame 'tool0'
//!     let frame_id = model.get_frame_id("tool0", None).unwrap();
//!     // we don't specify a frame type (None) as there is only one frame with this name
//!     let frame_placement = &data.frame_placements[frame_id];
//!     println!("Placement of frame 'tool0':\n{:?}", frame_placement);
//! }
//! ```
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
