import numpy as np
import dynamics as dyn
import time
import os

# set ROS_PACKAGE_PATH to find the example-robot-data package
os.environ["ROS_PACKAGE_PATH"] = (
    "examples/descriptions/example-robot-data:" + os.environ.get("ROS_PACKAGE_PATH", "")
)

# load an URDF file
robots_dir = "examples/descriptions/example-robot-data/robots/"
model, coll_model, viz_model = dyn.build_models_from_urdf(
    robots_dir + "ur_description/urdf/ur5_robot.urdf",
)

# Build a data frame associated with the model
data = model.create_data()

# sample a random joint configuration, joint velocities and accelerations
q = dyn.neutral(model)
q = q.to_numpy()

viz = dyn.visualize.MeshcatVisualizer(model, coll_model, viz_model)
viz.init_viewer(load_model=True)
viz.open()

while True:
    q += np.random.randn(model.nq) * 0.1
    viz.display(q)
    time.sleep(0.05)
