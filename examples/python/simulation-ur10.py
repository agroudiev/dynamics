import dynamics as dyn
import numpy as np
import time
import os

# set ROS_PACKAGE_PATH to find the example-robot-data package
os.environ["ROS_PACKAGE_PATH"] = (
    "examples/descriptions/example-robot-data:" + os.environ.get("ROS_PACKAGE_PATH", "")
)

# load an URDF file
robots_dir = "examples/descriptions/example-robot-data/robots/"
model, coll_model, viz_model = dyn.build_models_from_urdf(
    robots_dir + "ur_description/urdf/ur10_robot.urdf",
)

viz = dyn.visualize.MeshcatVisualizer(model, coll_model, viz_model)
viz.init_viewer(load_model=True, open=True)


# display a valid robot configuration.
q0 = dyn.neutral(model)
viz.display(q0)

input("Press any key to continue...")


# # play a bit with the simulation
dt = 0.01

data_sim = model.create_data()

t = 0.0
q = dyn.random_configuration(model)
v = np.zeros(model.nv)
tau_control = np.zeros(model.nv)
damping_value = 0.1

while True:
    tic = time.time()
    tau_control = -damping_value * v  # small damping
    a = dyn.aba(model, data_sim, q, v, tau_control)  # Forward dynamics

    # Semi-explicit integration
    v += a * dt
    q = dyn.integrate(model, q, v * dt)  # Configuration integration

    viz.display(q)
    toc = time.time()
    ellapsed = toc - tic

    dt_sleep = max(0, dt - (ellapsed))
    time.sleep(dt_sleep)
    t += dt
