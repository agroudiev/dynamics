import dynamics as dyn
import dynamics.collider as col
import numpy as np

model = dyn.Model()
geom_model = dyn.GeometryModel()

parent_id = 0

base_radius = 0.2
shape_base = col.Sphere(base_radius)
geom_base = dyn.GeometryObject("base", 0, 0, shape_base, dyn.SE3.Identity())
geom_base.mesh_color = np.array([1.0, 0.1, 0.1, 1.0])
geom_model.add_geometry_object(geom_base)

joint_placement = dyn.SE3.Identity()
body_mass = 1.0
body_radius = 0.1

joint_name = "joint"
joint_id = model.add_joint(
    parent_id, dyn.JointModelRX(), joint_placement, joint_name
)