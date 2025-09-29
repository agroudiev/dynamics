import numpy as np
import dynamics as dyn

# load an URDF file
model, geom_model = dyn.build_models_from_urdf("examples/descriptions/ur5_robot.urdf")

# Build a data frame associated with the model
data = model.create_data()

# sample a random joint configuration, joint velocities and accelerations
q = dyn.random_configuration(model)
v = np.random.rand(model.nv, 1)
a = np.random.rand(model.nv, 1)

# computes the inverse dynamics using Recursive Newton-Euler Algorithm (RNEA)
# tau = dyn.rnea(model, data, q, v, a)

# print("Joint torques: " + str(tau))