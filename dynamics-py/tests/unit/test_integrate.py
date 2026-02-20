import unittest
import dynamics as dyn
import pinocchio as pin
import numpy as np
from utils import (
    assert_models_equals,
    set_ros_package_path,
    EXAMPLE_ROBOT_DATA_URDFS,
)
import parameterized


def compare_urdf_integrate(test_case, file_path, mesh_dir=None):
    dyn_model, _, _ = dyn.build_models_from_urdf(file_path, mesh_dir)
    pin_model, _, _ = pin.buildModelsFromUrdf(file_path, mesh_dir)

    np.random.seed(0)
    q = pin.randomConfiguration(pin_model)
    v = np.random.rand(dyn_model.nv)

    dyn_q_next = dyn.integrate(dyn_model, q, v)
    pin_q_next = pin.integrate(pin_model, q, v)
    test_case.assertTrue(np.linalg.norm(dyn_q_next.to_numpy() - pin_q_next) < 1e-10)


class TestIntegrate(unittest.TestCase):
    def test_integrate_empty(self):
        # Create two empty models
        dyn_model = dyn.Model()
        pin_model = pin.Model()
        assert_models_equals(self, dyn_model, pin_model)

        q = np.zeros(0)
        v = np.zeros(0)

        dyn_q_next = dyn.integrate(dyn_model, q, v)
        pin_q_next = pin.integrate(pin_model, q, v)
        self.assertTrue(np.linalg.norm(dyn_q_next.to_numpy() - pin_q_next) < 1e-10)

    def test_integrate_one_joint(self):
        # Create two empty models
        dyn_model = dyn.Model()
        pin_model = pin.Model()

        # Add a revolute joint to both models
        np.random.seed(0)
        pin_placement = pin.SE3.Random()
        dyn_placement = dyn.SE3(
            rotation=pin_placement.rotation, translation=pin_placement.translation
        )

        dyn_model.add_joint(
            parent_id=0,
            joint_model=dyn.JointModelRZ(),
            placement=dyn_placement,
            name="joint1",
        )
        pin_model.addJoint(
            0,
            pin.JointModelRZ(),
            pin_placement,
            "joint1",
        )

        assert_models_equals(self, dyn_model, pin_model)

        np.random.seed(0)
        q = np.random.rand(1)
        v = np.random.rand(1)
        dyn_q_next = dyn.integrate(dyn_model, q, v)
        pin_q_next = pin.integrate(pin_model, q, v)
        self.assertTrue(np.linalg.norm(dyn_q_next.to_numpy() - pin_q_next) < 1e-10)

    def test_integrate_double_pendulum(self):
        compare_urdf_integrate(
            self, "examples/descriptions/double_pendulum_simple.urdf"
        )

    @parameterized.parameterized.expand(EXAMPLE_ROBOT_DATA_URDFS)
    def test_integrate_example_robot_data(self, path):
        robots_dir = "examples/descriptions/example-robot-data/robots/"
        set_ros_package_path("example-robot-data")
        compare_urdf_integrate(self, robots_dir + path)
