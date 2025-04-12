from . import dynamics
import dynamics.collider as collider
import meshcat
import meshcat.geometry as mg
import numpy as np
from enum import Enum

class GeometryType(Enum):
    COLLISION = 1
    VISUAL = 2

class MeshcatVisualizer:
    """A `dynamics` visualizer using Meshcat."""
    def __init__(
        self,
        model: dynamics.Model | None = dynamics.Model(),
        collision_model: dynamics.GeometryModel | None = None,
        visual_model: dynamics.GeometryModel | None = None,
    ):
        # Check if the arguments are valid
        if collision_model is not None:
            assert isinstance(collision_model, dynamics.GeometryModel), \
                "collision_model must be a GeometryModel"
        if visual_model is not None:
            assert isinstance(visual_model, dynamics.GeometryModel), \
                "visual_model must be a GeometryModel"
        self.model = model
        self.collision_model = collision_model
        self.visual_model = visual_model

    def init_viewer(self,
            viewer: meshcat.Visualizer | None = None,
            open: bool = False,
            load_model: bool = False,
            url: str | None = None
        ):
        """Start a new meschat server and client."""

        self.viewer = meshcat.Visualizer(url) if viewer is None else viewer

        if open:
            self.viewer.open()

        if load_model:
            self.load_model()

    def load_shape(self, geometry: collider.PyShapeWrapper, geometry_type: GeometryType):
        object = None

        # Cylinders need to be rotated
        R = np.array(
            [
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, -1.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
        RotatedCylinder = type(
            "RotatedCylinder", (mg.Cylinder,), {"intrinsic_transform": lambda _self: R}
        )
        
        if str(geometry.shape_type) == "ShapeType.Capsule": # quick fix until the enum comparison is fixed
            object = RotatedCylinder(2.0 * geometry.half_length, geometry.radius)
        elif str(geometry.shape_type) == "ShapeType.Cylinder":
            object = RotatedCylinder(2.0 * geometry.half_length, geometry.radius)
        elif str(geometry.shape_type) == "ShapeType.Cone":
            object = RotatedCylinder(2.0 * geometry.half_length, 0, geometry.radius, 0)
        elif str(geometry.shape_type) == "ShapeType.Cuboid":
            object = mg.Box(tuple((2.0 * geometry.half_extents).tolist()))
        elif str(geometry.shape_type) == "ShapeType.Sphere":
            object = mg.Sphere(geometry.radius)
        else:
            raise TypeError("geometry type not supported for visualization (type: {})".format(geometry.shape_type))

        return object

    def load_viewer_geometry_object(self, geometry_object: dynamics.GeometryObject, geometry_type: GeometryType):
        geometry = geometry_object.geometry
        meshcat_node = self.viewer[geometry_object.name]

        if isinstance(geometry, collider.PyShapeWrapper):
            object = self.load_shape(geometry, geometry_type)

            if isinstance(object, (mg.Geometry, mg.ReferenceSceneElement)):
                # add information for placement and color
                material = mg.MeshPhongMaterial()

                def to_material_color(rgba) -> int:
                    """Convert rgba color as list into rgba color as int"""
                    return (
                        int(rgba[0] * 255) * 256**2
                        + int(rgba[1] * 255) * 256
                        + int(rgba[2] * 255)
                    )
            
                mesh_color = geometry_object.mesh_color
                material.color = to_material_color(mesh_color)

                # add transparency if needed
                if float(mesh_color[3]) != 1.0:
                    material.transparent = True
                    material.opacity = float(mesh_color[3])

                meshcat_node.set_object(object, material)
            else:
                # just add the object to the viewer
                meshcat_node.set_object(object)
        else:
            raise NotImplementedError("geometry object is not a standard shape, cannot load it into viewer (type: {})".format(type(geometry)))

    def load_model(self):
        if self.visual_model is not None:
            for visual in self.visual_model.geometry_objects:
                self.load_viewer_geometry_object(visual, GeometryType.VISUAL)

    def reset(self):
        self.viewer.delete()

    def set_cam_target(self, target: np.ndarray):
        self.viewer.set_cam_target(target)

    def set_cam_pos(self, position: np.ndarray):
        self.viewer.set_cam_pos(position)

    def open(self):
        self.viewer.open()