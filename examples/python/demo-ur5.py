import numpy as np
import dynamics as dyn
import time

# load an URDF file
model, geom_model = dyn.build_models_from_urdf("examples/descriptions/ur5/ur5_robot.urdf")

# Build a data frame associated with the model
data = model.create_data()

# sample a random joint configuration, joint velocities and accelerations
q = dyn.neutral(model)
q = q.to_numpy()

viz = dyn.visualize.MeshcatVisualizer(model, geom_model, geom_model)
viz.init_viewer(load_model=True)
viz.open()

while True:
    q += np.array([0.0, 0.1, 0.0, 0.0, 0.0, 0.0])
    viz.display(q)
    time.sleep(0.05)