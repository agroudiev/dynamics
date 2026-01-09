import dynamics
import numpy as np

# create a new isometry from a rotation matrix and a translation vector
aMb = dynamics.SE3(np.eye(3), np.array([1.0, 2.0, 3.0]))
assert (aMb.rotation == np.eye(3)).all()
assert (aMb.translation == np.array([1.0, 2.0, 3.0])).all()

# identity isometry
M = dynamics.SE3.Identity()
assert (M.rotation == np.eye(3)).all()
assert (M.translation == np.array([0.0, 0.0, 0.0])).all()

# random isometry (coefficients uniformly sampled in [-1, 1])
r = dynamics.SE3.Random()

# inverse isometry
rotation = np.zeros((3, 3))
rotation[0, 2] = rotation[1, 1] = rotation[2, 0] = 1.0
aMb = dynamics.SE3(rotation, np.array([1.0, 2.0, 3.0]))
bMa = aMb.inverse()
assert (bMa.rotation == rotation).all()
assert (
    bMa.translation == np.array([-3.0, -2.0, -1.0])
).all()  # rotation has been applied

# compose isometries
aMb = dynamics.SE3(rotation, np.array([1.0, 2.0, 3.0]))
bMc = dynamics.SE3(rotation, np.array([4.0, 5.0, 6.0]))
cMb = aMb * bMc
assert (cMb.rotation == np.eye(3)).all()
assert (cMb.translation == np.array([1.0 + 6.0, 5.0 + 2.0, 3.0 + 4.0])).all()
