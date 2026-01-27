import unittest
import dynamics as dyn
import pinocchio as pin
import numpy as np
from utils import (
    assert_models_equals,
    assert_datas_equals,
    set_ros_package_path,
    EXAMPLE_ROBOT_DATA_URDFS,
)
import parameterized


def compare_urdf_fk(test_case, file_path, mesh_dir=None):
    dyn_model, _, _ = dyn.build_models_from_urdf(file_path, mesh_dir)
    pin_model, _, _ = pin.buildModelsFromUrdf(file_path, mesh_dir)

    dyn_data = dyn.Data(dyn_model)
    pin_data = pin.Data(pin_model)

    np.random.seed(0)
    q = pin.randomConfiguration(pin_model)  # do not use np.random.rand
    dyn.forward_kinematics(dyn_model, dyn_data, q)
    pin.forwardKinematics(pin_model, pin_data, q)
    assert_datas_equals(test_case, dyn_data, pin_data)

    dyn.update_frame_placements(dyn_model, dyn_data)
    pin.updateFramePlacements(pin_model, pin_data)
    assert_datas_equals(test_case, dyn_data, pin_data)


class TestForwardKinematics(unittest.TestCase):
    def test_fk_empty(self):
        # Create two empty models
        dyn_model = dyn.Model()
        pin_model = pin.Model()
        assert_models_equals(self, dyn_model, pin_model)

        # Create data for both models
        dyn_data = dyn.Data(dyn_model)
        pin_data = pin.Data(pin_model)
        assert_datas_equals(self, dyn_data, pin_data)

        # Check forward kinematics
        dyn.forward_kinematics(dyn_model, dyn_data, np.zeros(dyn_model.nq))
        pin.forwardKinematics(pin_model, pin_data, np.zeros(pin_model.nq))
        assert_datas_equals(self, dyn_data, pin_data)

    def test_fk_one_joint(self):
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

        # Create data for both models
        dyn_data = dyn.Data(dyn_model)
        pin_data = pin.Data(pin_model)
        assert_datas_equals(self, dyn_data, pin_data)

        # Check forward kinematics
        np.random.seed(0)
        q = np.random.rand(dyn_model.nq)
        dyn.forward_kinematics(dyn_model, dyn_data, q)
        pin.forwardKinematics(pin_model, pin_data, q)
        assert_datas_equals(self, dyn_data, pin_data)

    @parameterized.parameterized.expand(EXAMPLE_ROBOT_DATA_URDFS)
    def test_fk_example_robot_data(self, path):
        robots_dir = "examples/descriptions/example-robot-data/robots/"
        set_ros_package_path("example-robot-data")
        compare_urdf_fk(self, robots_dir + path)
