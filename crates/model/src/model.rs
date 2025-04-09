//! The standard `Model`.

use nalgebra::Isometry3;
use std::collections::HashMap;

/// A `Model` is a data structure that contains the information about the robot model,
/// including the joints models, placements, the link inertias, and the frames.
pub struct Model {
    /// The name of the model.
    pub name: String,
    /// The names of the joints.
    pub joint_names: HashMap<usize, String>,
    /// The placements of the joints.
    pub joint_placements: HashMap<usize, Isometry3<f32>>,
    /// The number of position variables.
    pub nq: usize,
    /// The number of velocity variables.
    pub nv: usize,
}

impl Model {
    /// Creates a new `Model` with the given parameters.
    ///
    /// # Arguments
    ///
    /// * `name` - The name of the model.
    /// * `joint_names` - The names of the joints.
    /// * `joint_placements` - The placements of the joints.
    /// * `nq` - The number of position variables.
    /// * `nv` - The number of velocity variables.
    pub fn new(
        name: String,
        joint_names: HashMap<usize, String>,
        joint_placements: HashMap<usize, Isometry3<f32>>,
        nq: usize,
        nv: usize,
    ) -> Self {
        Self {
            name,
            joint_names,
            joint_placements,
            nq,
            nv,
        }
    }
}
