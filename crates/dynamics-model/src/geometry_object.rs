//! Geometry object structure, representing a primitive with visualization properties and collision properties.

use std::fmt::Debug;

use collider_rs::shape::ShapeWrapper;
use dynamics_spatial::{color::Color, se3::SE3};

/// Data structure that contains the information about the geometry object,
/// used for visualization, collision detection and distance computation.
pub struct GeometryObject {
    /// The name of the geometry object.
    pub name: String,
    /// The identifier of the parent joint.
    pub parent_joint: usize,
    /// The identifier of the parent frame.
    pub parent_frame: usize,
    /// Whether to disable collision detection and distance check for this object.
    pub disable_collision: bool,
    /// The `collider` geometry object.
    pub geometry: ShapeWrapper,
    /// The RGBA color of the mesh.
    pub mesh_color: Color,
    /// The placement of the geometry object in the parent frame.
    pub placement: SE3,
}

impl GeometryObject {
    /// Creates a new `GeometryObject` with the given parameters.
    ///
    /// # Arguments
    ///
    /// * `name` - The name of the geometry object.
    /// * `parent_joint` - The identifier of the parent joint.
    /// * `geometry` - The `collider` shape of the geometry object (used for collisions).
    /// * `mesh_color` - The RGBA color of the mesh.
    /// * `placement` - The placement of the geometry object in the parent frame.
    #[must_use]
    pub fn new(
        name: String,
        parent_joint: usize,
        parent_frame: usize,
        geometry: ShapeWrapper,
        mesh_color: Color,
        placement: SE3,
    ) -> Self {
        Self {
            name,
            parent_joint,
            parent_frame,
            disable_collision: false,
            geometry,
            mesh_color,
            placement,
        }
    }
}

impl Clone for GeometryObject {
    fn clone(&self) -> Self {
        Self {
            name: self.name.clone(),
            parent_joint: self.parent_joint,
            parent_frame: self.parent_frame,
            disable_collision: self.disable_collision,
            geometry: self.geometry.clone_box(),
            mesh_color: self.mesh_color,
            placement: self.placement,
        }
    }
}

impl Debug for GeometryObject {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("GeometryObject")
            .field("name", &self.name)
            .field("parent_joint", &self.parent_joint)
            .field("parent_frame", &self.parent_frame)
            .field("disable_collision", &self.disable_collision)
            // .field("geometry", &self.geometry)
            .field("mesh_color", &self.mesh_color)
            .field("placement", &self.placement)
            .finish()
    }
}
