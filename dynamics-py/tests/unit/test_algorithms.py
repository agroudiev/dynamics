import unittest
import dynamics as dyn
import pinocchio as pin
import numpy as np
from utils import assert_models_equals, assert_datas_equals


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
