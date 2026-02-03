//! Defines spatial **forces** and related operations.

use crate::motion::SpatialMotion;

/// Spatial force vector, combining torque and force components.
///
/// A spatial force is represented as a 6-dimensional vector,
/// which can be decomposed into $\begin{bmatrix} \tau & f \end{bmatrix}$,
/// where $\tau$ is the torque (rotational component) and $f$ is the force (translational component).
pub type SpatialForce = SpatialMotion;
// TODO: refactor
