import dynamics as dyn
import dynamics.collider as col  # type: ignore
import numpy as np
import math

# import time

model = dyn.Model()
geom_model = dyn.GeometryModel()

parent_id = dyn.WORLD_ID

base_radius = 0.2
shape_base = col.Sphere(base_radius)
geom_base = dyn.GeometryObject("base", 0, 0, shape_base, dyn.SE3.Identity())
geom_base.mesh_color = np.array([1.0, 0.1, 0.1, 1.0])
geom_model.add_geometry_object(geom_base)

joint_placement = dyn.SE3.Identity()
body_mass = 1.0
body_radius = 0.1

joint_name = "joint"
joint_id = model.add_joint(parent_id, dyn.JointModelRX(), joint_placement, joint_name)

body_inertia = dyn.Inertia.FromSphere(body_mass, body_radius)
body_placement = joint_placement.copy()
# body_placement.translation[2] = 1.0
body_placement.translation = np.array([0.0, 0.0, 1.0])
model.append_body_to_joint(joint_id, body_inertia, body_placement)

geom1_name = "ball"
shape1 = col.Sphere(body_radius)
geom1_obj = dyn.GeometryObject(geom1_name, joint_id, 0, shape1, body_placement)
geom1_obj.mesh_color = np.ones(4)
geom_model.add_geometry_object(geom1_obj)

geom2_name = "bar"
shape2 = col.Cylinder(body_radius / 4.0, body_placement.translation[2])
shape2_placement = body_placement.copy()
# shape2_placement.translation[2] /= 2.0
shape2_placement.translation = np.array([0.0, 0.0, 0.5])

geom2_obj = dyn.GeometryObject(geom2_name, joint_id, 0, shape2, shape2_placement)
geom2_obj.mesh_color = np.array([0.0, 0.0, 0.0, 1.0])
geom_model.add_geometry_object(geom2_obj)

parent_id = joint_id
joint_placement = body_placement.copy()


# initialize the viewer
visual_model = geom_model
viz = dyn.visualize.MeshcatVisualizer(model, geom_model, visual_model)
viz.init_viewer(load_model=True)


# display a valid robot configuration.
q0 = dyn.neutral(model)
viz.display(q0)

input("Press any key to continue...")


# # play a bit with the simulation
dt = 0.01
T = 5
N = math.floor(T / dt)

# model.lower_position_limit.fill(-math.pi)
# model.upper_position_limit.fill(+math.pi)

data_sim = model.create_data()

t = 0.0
q = dyn.random_configuration(model)
print(q)
v = np.zeros(model.nv)
tau_control = np.zeros(model.nv)
damping_value = 0.1

# for k in range(N):
#     tic = time.time()
#     tau_control = -damping_value * v  # small damping
#     a = dyn.aba(model, data_sim, q, v, tau_control)  # Forward dynamics

#     # Semi-explicit integration
#     v += a * dt
#     q = dyn.integrate(model, q, v * dt)  # Configuration integration

#     viz.display(q)
#     toc = time.time()
#     ellapsed = toc - tic

#     dt_sleep = max(0, dt - (ellapsed))
#     time.sleep(dt_sleep)
#     t += dt
