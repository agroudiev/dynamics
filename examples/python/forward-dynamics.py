import dynamics as dyn
import numpy as np

# Load a robot model from a URDF file
model, _coll_model, _viz_model = dyn.build_models_from_urdf(
    "./examples/descriptions/ur5/ur5_robot.urdf",
    "./examples/descriptions/ur5/",  # mesh directory
)
data = model.create_data()

# Define a random configuration, velocity and torque
q = dyn.random_configuration(model)
v = np.random.rand(model.nv)
tau = np.random.rand(model.nv)

# Compute forward dynamics
ddq = dyn.forward_dynamics(model, data, q, v, tau)
print("Joint accelerations:", str(ddq))
