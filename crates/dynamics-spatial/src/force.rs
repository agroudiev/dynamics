//! Defines spatial **forces** and related operations.

use crate::{vector3d::Vector3D, vector6d::Vector6D};

/// Spatial force vector, combining torque and force components.
///
/// A spatial force is represented as a 6-dimensional vector,
/// which can be decomposed into $\begin{bmatrix} \tau & f \end{bmatrix}$,
/// where $\tau$ is the torque (rotational component) and $f$ is the force (translational component).
pub type SpatialForce = Vector6D;

impl SpatialForce {
    /// Creates a new `SpatialForce` from the given torque and force components.
    ///
    /// # Arguments
    ///
    /// * `torque` - The torque component (3D vector).
    /// * `force` - The force component (3D vector).
    #[must_use]
    pub fn from_components(torque: Vector3D, force: Vector3D) -> Self {
        let mut data = [0.0; 6];
        data[0..3].copy_from_slice(torque.as_slice());
        data[3..6].copy_from_slice(force.as_slice());
        Self::from_slice(&data)
    }
}
