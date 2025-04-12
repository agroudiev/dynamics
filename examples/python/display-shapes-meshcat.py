import dynamics
import dynamics.collider as collider
import numpy as np

geometries = [
    collider.Capsule(0.1, 0.8),
    collider.Sphere(0.5),
    collider.Cuboid((1, 1, 1)),
    collider.Cylinder(0.1, 1.0),
    collider.Cone(0.5, 1.0),
]

model = dynamics.Model()

geom_model = dynamics.GeometryModel()
for i, geom in enumerate(geometries):
    placement = dynamics.SE3(np.eye(3), np.array([i, 0.0, 0.0]))
    geom_obj = dynamics.GeometryObject(f"obj{i}", 0, 0, geom, placement)

# viz = dynamics.visualize.MeshcatVisualizer(model, geom_model, geom_model)
# viz.init_viewer(open=True, load_model=True)

# input("Press enter to continue...")