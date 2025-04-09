use collider::shape::Shape;
use nalgebra::Vector4;

pub struct GeometryObject {
    /// Whether to disable collision detection and distance check for this object.
    pub disable_collision: bool,
    /// The `collider` geometry object.
    pub geometry: Shape,
    /// The RGBA color of the mesh.
    pub mesh_color: Vector4<f32>,
}

impl GeometryObject {
    /// Creates a new `GeometryObject` with the given parameters.
    ///
    /// # Arguments
    ///
    /// * `disable_collision` - Whether to disable collision detection and distance check for this object.
    /// * `geometry` - The `collider` geometry object.
    /// * `mesh_color` - The RGBA color of the mesh.
    pub fn new(disable_collision: bool, geometry: Shape, mesh_color: Vector4<f32>) -> Self {
        Self {
            disable_collision,
            geometry,
            mesh_color,
        }
    }
}
