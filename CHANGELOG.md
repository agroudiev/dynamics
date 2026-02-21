# Changelog

This file is used to track changes made to the project over time.

## [Unreleased]
### Added
#### Algorithms
- Added `forward_dynamics` function implementing the ABA algorithm for forward dynamics computation.

#### Model
- Added `append_body_to_joint` method to the model to append add inertias to joints with respect to its placement.

#### Joints
- Python binding for revolute unaligned joints (`JointModelRevoluteUnaligned`).
- Refactor the `get_axis` method signature to return a reference to the joint axis.
- Refactor the data creation methods.
- Refactor error handling.

#### Spatial
- Added `action_matrix`, `dual_matrix`, and `inv_matrix` methods to SE3 transforms to compute the action, dual action, and inverse action matrices.
- Added `InertiaMatrix` type as an alternative representation of spatial inertias.
- Add more bindings for inertias.

### Fixes and improvements
- Better handling of infinite configurations for joint subspaces.
- Better display for spatial types.
- Fix the way that skew matrices are computed.
- Fix the way that inertia matrices are computed.
- Implemented `__array__` method for spatial types to allow simpler conversion to NumPy arrays in Python.

## [0.0.2] - 2026-02-08
### Added
### Algorithms
- Added `update_frame_placements` algorithm to update frame placements based on joint placements.
- Added optional velocity and acceleration parameters to `forward_kinematics` function.
- Added `frames_forward_kinematics` function to compute forward kinematics for all frames in the model.
- Added `inverse_dynamics` function implementing the RNEA algorithm for inverse dynamics computation.

### Model & Data
- Data now stores joint velocities and accelerations.
- Added a `print_joint_tree` method to visualize the joint hierarchy.

### Joints
- Python bindings for continuous joints (`JointModelRUB*`) and prismatic joints (`JointModelP*`).
- Joint data now stores the joint configurations and velocity vector, as well as the joint velocities and accelerations.
- Compute joint subspace constraint.
- Compute joint bias (Coriolis and centrifugal effects).

### Spatial
- Added Python bindings for `SpatialMotion` type.
- More efficient implementation of the `cross` method for `SpatialMotion`.
- Diverse improvements to the bindings and display of spatial types.

### Misc
- Fix docs.rs build for all crates
- Add `python` feature to workspace and all crates for building Python bindings
- Add `prelude` module to re-export commonly used items for easier library usage.
- Added benchmarks for URDF parsing, forward kinematics, and inverse dynamics.

## [0.0.1] - 2026-01-25
### Added
#### Spatial
- The following spatial quantity types have been implemented:
  - SE3 Transforms
  - Vector3D, Vector6D
  - Symmetric3 Matrices
  - Spatial Motion Vector
  - Spatial Force Vector
  - Spatial Inertia Matrix

#### Model
- Initial model implementation
- Support for frames, joints, inertias

#### Joints
- The following joint types have been implemented:
  - Continuous Joint
  - Fixed Joint
  - Prismatic Joint
  - Revolute Joint

### Parser
- Initial URDF parser implementation
- Tested on every model from [`example-robot-data`](https://github.com/Gepetto/example-robot-data/)

#### Algorithms
- Experimental implementations of
  - Forward dynamics
  - RNEA for inverse dynamics

#### Visualization
- Meshcat-based visualization support