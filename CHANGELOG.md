# Changelog

This file is used to track changes made to the project over time.

## [Unreleased]
### Added
#### Joints
- Python bindings for continuous joints (`JointModelRU*`) and prismatic joints (`JointModelP*`).

## [0.0.1] - 2025-01-25
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