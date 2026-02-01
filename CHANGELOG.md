# Changelog

This file is used to track changes made to the project over time.

## [Unreleased]
### Added
### Algorithms
- Added `update_frame_placements` algorithm to update frame placements based on joint placements.
- Added optional velocity and acceleration parameters to `forward_kinematics` function.

### Model & Data
- Data now stores joint velocities and accelerations.

### Joints
- Python bindings for continuous joints (`JointModelRUB*`) and prismatic joints (`JointModelP*`).
- Joint data now stores the joint configurations and velocity vector, as well as the joint velocities and accelerations.
- Compute joint subspace constraint.

### Spatial
- Added Python bindings for `SpatialMotion` type.

### Misc
- Fix docs.rs build for all crates
- Add `python` feature to workspace and all crates for building Python bindings

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