import dynamics
import numpy as np

# === SE3 ===
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


# === Symmetric3 ===
# create a new symmetric matrix from its coefficients
s = dynamics.Symmetric3(1.0, 2.0, 3.0, 0.1, 0.2, 0.3)
assert (
    s.matrix() == np.array([[1.0, 0.1, 0.2], [0.1, 2.0, 0.3], [0.2, 0.3, 3.0]])
).all()

# zero symmetric matrix
s_zero = dynamics.Symmetric3.Zero()
assert (s_zero.matrix() == np.zeros((3, 3))).all()

# identity symmetric matrix
s_identity = dynamics.Symmetric3.Identity()
assert (s_identity.matrix() == np.eye(3)).all()

# linear combinations
s1 = dynamics.Symmetric3(1.0, 2.0, 3.0, 0.1, 0.2, 0.3)
s2 = dynamics.Symmetric3(4.0, 5.0, 6.0, 0.4, 0.5, 0.6)
s_sum = 3.0 * s1 - s2 * 2.0
assert (s_sum.matrix() == 3.0 * s1.matrix() - s2.matrix() * 2.0).all()

# element access
s = dynamics.Symmetric3(1.0, 2.0, 3.0, 0.1, 0.2, 0.3)
assert s[0, 1] == 0.1
assert s[1, 0] == 0.1

# vector multiplication
s = dynamics.Symmetric3(1.0, 2.0, 3.0, 0.1, 0.2, 0.3)
v = dynamics.Vector3D(1.0, 2.0, 3.0)
sv = s * v
assert (sv.vector() == s.matrix() @ v.vector()).all()

# === Spatial Motion ===
# create a new spatial motion from its angular and linear parts
w = dynamics.Vector3D(1.0, 2.0, 3.0)
v = dynamics.Vector3D(4.0, 5.0, 6.0)
motion = dynamics.SpatialMotion(w, v)

# or create it using two numpy arrays
w_np = np.array([1.0, 2.0, 3.0])
v_np = np.array([4.0, 5.0, 6.0])
motion = dynamics.SpatialMotion.from_parts(w_np, v_np)
assert (motion.rotation.vector() == w_np).all()
assert (motion.translation.vector() == v_np).all()
