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

    def test_cross_motion(self):
        np.random.seed(0)
        motion_vec1 = np.random.uniform(-1.0, 1.0, 6)
        motion_vec2 = np.random.uniform(-1.0, 1.0, 6)

        motion_dyn1 = dyn.SpatialMotion(motion_vec1)
        motion_pin1 = pin.Motion(motion_vec1)

        motion_dyn2 = dyn.SpatialMotion(motion_vec2)
        motion_pin2 = pin.Motion(motion_vec2)

        cross_dyn = motion_dyn1.cross(motion_dyn2)
        cross_pin = motion_pin1.cross(motion_pin2)

        self.assertTrue(np.linalg.norm(cross_dyn.to_numpy() - cross_pin.vector) < 1e-14)

    def test_action_matrices(self):
        np.random.seed(0)
        rotation = np.random.uniform(-1.0, 1.0, (3, 3))
        rotation, _ = np.linalg.qr(rotation)
        translation = np.random.uniform(-1.0, 1.0, 3)

        M_dyn = dyn.SE3(rotation, translation)
        M_pin = pin.SE3(rotation, translation)

        action_dyn = M_dyn.action_matrix()
        action_pin = M_pin.toActionMatrix()

        self.assertTrue(np.linalg.norm(action_dyn - action_pin) < 1e-14)

        dual_action_dyn = M_dyn.dual_matrix()
        dual_action_pin = M_pin.toDualActionMatrix()

        self.assertTrue(np.linalg.norm(dual_action_dyn - dual_action_pin) < 1e-14)

        inv_action_dyn = M_dyn.inv_matrix()
        inv_action_pin = M_pin.toActionMatrixInverse()

        self.assertTrue(np.linalg.norm(inv_action_dyn - inv_action_pin) < 1e-14)

    def test_transform_frame(self):
        np.random.seed(0)
        rotation = np.random.uniform(-1.0, 1.0, (3, 3))
        rotation, _ = np.linalg.qr(rotation)
        translation = np.random.uniform(-1.0, 1.0, 3)

        M_dyn = dyn.SE3(rotation, translation)
        M_pin = pin.SE3(rotation, translation)

        mass = 10.0
        lever = np.random.uniform(-1.0, 1.0, 3) * 10
        inertia = np.random.uniform(-1.0, 1.0, (3, 3))
        inertia = (inertia + inertia.T) / 2 + 10 * np.eye(3)
        pin_inertia = pin.Inertia(mass, lever, inertia)
        dyn_inertia = dyn.Inertia(mass, lever, inertia)
        assert_inertias_equals(self, dyn_inertia, pin_inertia)

        dyn_dual_matrix = M_dyn.dual_matrix()
        pin_dual_matrix = M_pin.toDualActionMatrix()
        self.assertTrue(np.linalg.norm(dyn_dual_matrix - pin_dual_matrix) < 1e-14)

        dyn_inverse_matrix = M_dyn.inv_matrix()
        pin_inv_matrix = M_pin.toActionMatrixInverse()
        self.assertTrue(np.linalg.norm(dyn_inverse_matrix - pin_inv_matrix) < 1e-14)

        dyn_inertia_matrix = dyn_inertia.matrix()
        pin_inertia_matrix = pin_inertia.matrix()
        self.assertTrue(np.linalg.norm(dyn_inertia_matrix - pin_inertia_matrix) < 1e-14)

        dyn_trans = dyn_inertia.transform_frame(M_dyn)
        pin_trans = (
            M_pin.toDualActionMatrix()
            @ pin_inertia.matrix()
            @ M_pin.toActionMatrixInverse()
        )
        self.assertTrue(np.linalg.norm(dyn_trans - pin_trans) < 1e-14)
