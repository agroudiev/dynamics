import unittest
import dynamics as dyn
import pinocchio as pin
import numpy as np
from utils import assert_inertias_equals


class TestSpatial(unittest.TestCase):
    def test_identity_se3(self):
        M_dyn = dyn.SE3.Identity()
        M_pin = pin.SE3.Identity()

        self.assertTrue((M_dyn.rotation == M_pin.rotation).all())
        self.assertTrue((M_dyn.translation == M_pin.translation).all())

    def test_random_se3(self):
        np.random.seed(0)
        rotation = np.random.uniform(-1.0, 1.0, (3, 3))
        rotation, _ = np.linalg.qr(rotation)
        translation = np.random.uniform(-1.0, 1.0, 3)

        M_dyn = dyn.SE3(rotation, translation)
        M_pin = pin.SE3(rotation, translation)

        self.assertTrue((M_dyn.rotation == M_pin.rotation).all())
        self.assertTrue((M_dyn.translation == M_pin.translation).all())

    def test_inverse_se3(self):
        np.random.seed(0)
        rotation = np.random.uniform(-1.0, 1.0, (3, 3))
        rotation, _ = np.linalg.qr(rotation)
        translation = np.random.uniform(-1.0, 1.0, 3)

        M_dyn = dyn.SE3(rotation, translation)
        M_pin = pin.SE3(rotation, translation)

        M_dyn_inv = M_dyn.inverse()
        M_pin_inv = M_pin.inverse()

        self.assertTrue((M_dyn_inv.rotation == M_pin_inv.rotation).all())
        self.assertTrue((M_dyn_inv.translation == M_pin_inv.translation).all())

    def test_compose_se3(self):
        np.random.seed(0)
        rotation1 = np.random.uniform(-1.0, 1.0, (3, 3))
        rotation1, _ = np.linalg.qr(rotation1)
        translation1 = np.random.uniform(-1.0, 1.0, 3)

        rotation2 = np.random.uniform(-1.0, 1.0, (3, 3))
        rotation2, _ = np.linalg.qr(rotation2)
        translation2 = np.random.uniform(-1.0, 1.0, 3)

        M_dyn1 = dyn.SE3(rotation1, translation1)
        M_pin1 = pin.SE3(rotation1, translation1)

        M_dyn2 = dyn.SE3(rotation2, translation2)
        M_pin2 = pin.SE3(rotation2, translation2)

        M_dyn_comp = M_dyn1 * M_dyn2
        M_pin_comp = M_pin1 * M_pin2

        self.assertTrue(
            np.linalg.norm(M_dyn_comp.rotation - M_pin_comp.rotation) < 1e-15
        )
        self.assertTrue(
            np.linalg.norm(M_dyn_comp.translation - M_pin_comp.translation) < 1e-15
        )

    def test_homogeneous_matrix_se3(self):
        np.random.seed(0)
        rotation = np.random.uniform(-1.0, 1.0, (3, 3))
        rotation, _ = np.linalg.qr(rotation)
        translation = np.random.uniform(-1.0, 1.0, 3)

        M_dyn = dyn.SE3(rotation, translation)
        M_pin = pin.SE3(rotation, translation)

        H_dyn = M_dyn.homogeneous
        H_pin = M_pin.homogeneous

        self.assertTrue(np.linalg.norm(H_dyn - H_pin) < 1e-15)

    def test_add_inertias(self):
        np.random.seed(0)
        pin_inertia1 = pin.Inertia.Random()
        dyn_inertia1 = dyn.Inertia(
            pin_inertia1.mass,
            pin_inertia1.lever,
            pin_inertia1.inertia,
        )

        pin_inertia2 = pin.Inertia.Random()
        dyn_inertia2 = dyn.Inertia(
            pin_inertia2.mass,
            pin_inertia2.lever,
            pin_inertia2.inertia,
        )

        pin_inertia_sum = pin_inertia1 + pin_inertia2
        dyn_inertia_sum = dyn_inertia1 + dyn_inertia2

        assert_inertias_equals(self, dyn_inertia_sum, pin_inertia_sum)

    def test_act_se3_motion(self):
        np.random.seed(0)
        rotation = np.random.uniform(-1.0, 1.0, (3, 3))
        rotation, _ = np.linalg.qr(rotation)
        translation = np.random.uniform(-1.0, 1.0, 3)

        M_dyn = dyn.SE3(rotation, translation)
        M_pin = pin.SE3(rotation, translation)

        self.assertTrue(np.linalg.norm(M_dyn.homogeneous - M_pin.homogeneous) < 1e-15)

        motion_vec = np.random.uniform(-1.0, 1.0, 6)

        motion_dyn = dyn.SpatialMotion(motion_vec)
        motion_pin = pin.Motion(motion_vec)

        self.assertTrue(
            np.linalg.norm(motion_dyn.to_numpy() - motion_pin.vector) < 1e-15
        )

        motion_dyn_transformed = M_dyn.act(motion_dyn)
        motion_pin_transformed = M_pin.act(motion_pin)

        self.assertTrue(
            np.linalg.norm(
                motion_dyn_transformed.to_numpy() - motion_pin_transformed.vector
            )
            < 1e-15
        )

        motion_dyn_transformed_inv = M_dyn.act_inv(motion_dyn)
        motion_pin_transformed_inv = M_pin.actInv(motion_pin)

        self.assertTrue(
            np.linalg.norm(
                motion_dyn_transformed_inv.to_numpy()
                - motion_pin_transformed_inv.vector
            )
            < 1e-15
        )
