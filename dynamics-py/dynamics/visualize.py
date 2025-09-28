from . import dynamics
import dynamics.collider as collider
import meshcat
import meshcat.geometry as mg
import numpy as np
from enum import Enum
from pathlib import Path

class GeometryType(Enum):
    COLLISION = 1
    VISUAL = 2

def create_capsule(length, radius, radial_resolution=30, cap_resolution=10):
    nbv = np.array([max(radial_resolution, 4), max(cap_resolution, 4)])
    h = length
    r = radius
    position = 0
    vertices = np.zeros((nbv[0] * (2 * nbv[1]) + 2, 3))
    for j in range(nbv[0]):
        phi = (2 * np.pi * j) / nbv[0]
        for i in range(nbv[1]):
            theta = (np.pi / 2 * i) / nbv[1]
            vertices[position + i, :] = np.array(
                [
                    np.cos(theta) * np.cos(phi) * r,
                    np.cos(theta) * np.sin(phi) * r,
                    -h / 2 - np.sin(theta) * r,
                ]
            )
            vertices[position + i + nbv[1], :] = np.array(
                [
                    np.cos(theta) * np.cos(phi) * r,
                    np.cos(theta) * np.sin(phi) * r,
                    h / 2 + np.sin(theta) * r,
                ]
            )
        position += nbv[1] * 2
    vertices[-2, :] = np.array([0, 0, -h / 2 - r])
    vertices[-1, :] = np.array([0, 0, h / 2 + r])
    indexes = np.zeros((nbv[0] * (4 * (nbv[1] - 1) + 4), 3))
    index = 0
    stride = nbv[1] * 2
    last = nbv[0] * (2 * nbv[1]) + 1
    for j in range(nbv[0]):
        j_next = (j + 1) % nbv[0]
        indexes[index + 0] = np.array(
            [
                j_next * stride + nbv[1],
                j_next * stride,
                j * stride,
            ]
        )
        indexes[index + 1] = np.array(
            [
                j * stride + nbv[1],
                j_next * stride + nbv[1],
                j * stride,
            ]
        )
        indexes[index + 2] = np.array(
            [
                j * stride + nbv[1] - 1,
                j_next * stride + nbv[1] - 1,
                last - 1,
            ]
        )
        indexes[index + 3] = np.array(
            [
                j_next * stride + 2 * nbv[1] - 1,
                j * stride + 2 * nbv[1] - 1,
                last,
            ]
        )
        for i in range(nbv[1] - 1):
            indexes[index + 4 + i * 4 + 0] = np.array(
                [
                    j_next * stride + i,
                    j_next * stride + i + 1,
                    j * stride + i,
                ]
            )
            indexes[index + 4 + i * 4 + 1] = np.array(
                [
                    j_next * stride + i + 1,
                    j * stride + i + 1,
                    j * stride + i,
                ]
            )
            indexes[index + 4 + i * 4 + 2] = np.array(
                [
                    j_next * stride + nbv[1] + i + 1,
                    j_next * stride + nbv[1] + i,
                    j * stride + nbv[1] + i,
                ]
            )
            indexes[index + 4 + i * 4 + 3] = np.array(
                [
                    j_next * stride + nbv[1] + i + 1,
                    j * stride + nbv[1] + i,
                    j * stride + nbv[1] + i + 1,
                ]
            )
        index += 4 * (nbv[1] - 1) + 4
    return mg.TriangularMeshGeometry(vertices, indexes)

class MeshcatVisualizer:
    """A `dynamics` visualizer using Meshcat."""
    def __init__(
        self,
        model: dynamics.Model | None = dynamics.Model(),
        collision_model: dynamics.GeometryModel | None = None,
        visual_model: dynamics.GeometryModel | None = None,
        data: dynamics.Data | None = None,
        collision_data: dynamics.GeometryData | None = None,
        visual_data: dynamics.GeometryData | None = None,
    ):
        # Check if the arguments are valid
        if collision_model is not None:
            assert isinstance(collision_model, dynamics.GeometryModel), \
                "collision_model must be a GeometryModel"
        if visual_model is not None:
            assert isinstance(visual_model, dynamics.GeometryModel), \
                "visual_model must be a GeometryModel"
        if data is not None:
            assert isinstance(data, dynamics.Data), \
                "data must be a Data object"
        if collision_data is not None:
            assert isinstance(collision_data, dynamics.GeometryData), \
                "collision_data must be a GeometryData object"
        if visual_data is not None:
            assert isinstance(visual_data, dynamics.GeometryData), \
                "visual_data must be a GeometryData object"
        self.model = model
        self.collision_model = collision_model
        self.visual_model = visual_model
        self.data = data
        self.collision_data = collision_data
        self.visual_data = visual_data

    def init_viewer(self,
            viewer: meshcat.Visualizer | None = None,
            open: bool = False,
            load_model: bool = False,
            url: str | None = None
        ):
        """Start a new meschat server and client."""

        self.viewer = meshcat.Visualizer(url) if viewer is None else viewer

        if self.data is None:
            self.data = dynamics.Data(self.model)
        if self.collision_data is None and self.collision_model is not None:
            self.collision_data = dynamics.GeometryData(self.data, self.collision_model)
        if self.visual_data is None and self.visual_model is not None:
            self.visual_data = dynamics.GeometryData(self.data, self.visual_model)

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
            if hasattr(mg, "TriangularMeshGeometry"):
                object = create_capsule(2.0 * geometry.half_length, geometry.radius)
            else:
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

    def load_mesh(self, geometry: collider.PyMesh):
        file_extension = Path(geometry.mesh_path).suffix
        if file_extension.lower() == ".dae":
            raise NotImplementedError("DAE mesh format not supported for visualization yet.")
            # obj = DaeMeshGeometry(geometry.mesh_path)
        elif file_extension.lower() == ".obj":
            obj = mg.ObjMeshGeometry.from_file(geometry.mesh_path)
        elif file_extension.lower() == ".stl":
            obj = mg.StlMeshGeometry.from_file(geometry.mesh_path)
        else:
            raise NotImplementedError("mesh format not supported for visualization (type: {})".format(file_extension))

        return obj

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
        elif isinstance(geometry, collider.PyMesh):
            object = self.load_mesh(geometry)
        else:
            raise NotImplementedError("geometry object is not a standard shape, cannot load it into viewer (type: {})".format(type(geometry)))

    def load_model(self):
        if self.visual_model is not None:
            for visual in self.visual_model.geometry_objects:
                self.load_viewer_geometry_object(visual, GeometryType.VISUAL)

        self.display_visuals(True)

    def reset(self):
        self.viewer.delete()

    def set_cam_target(self, target: np.ndarray):
        self.viewer.set_cam_target(target)

    def set_cam_pos(self, position: np.ndarray):
        self.viewer.set_cam_pos(position)

    def open(self):
        self.viewer.open()

    def display_visuals(self, visibility: bool):
        """Set whether to display visual objects or not."""

        if self.visual_model is None:
            self.display_visuals = False
        else:
            self.display_visuals = visibility

        if self.display_visuals:
            self.update_placements(GeometryType.VISUAL)

    def update_placements(self, geometry_type: GeometryType):
        """Update the placements of the geometry objects in the viewer."""
        if geometry_type == GeometryType.VISUAL:
            geom_model = self.visual_model
            geom_data = self.visual_data
        else:
            geom_model = self.collision_model
            geom_data = self.collision_data
        
        geom_data.update_geometry_data(self.data, geom_model)
        for object in geom_model.geometry_objects:
            # placement in world frame
            placement = geom_data.get_object_placement(object.id)
            T = placement.homogeneous
            self.viewer[object.name].set_transform(T)

    def display(self, q: np.ndarray | None = None):
        """Display the robot in the given configuration."""

        if q is not None:
            dynamics.forward_kinematics(self.model, self.data, q)

        self.update_placements(GeometryType.VISUAL)