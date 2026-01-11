import unittest
import dynamics as dyn
import pinocchio as pin
import numpy as np
from utils import assert_models_equals, assert_datas_equals


class TestModel(unittest.TestCase):
    def test_empty_model(self):
        assert_models_equals(self, dyn.Model(), pin.Model())

    def test_add_joint(self):
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

    def test_add_frame(self):
        # Create two empty models
        dyn_model = dyn.Model()
        pin_model = pin.Model()

        # Add a frame of each type to both models
        np.random.seed(0)
        for i, types in enumerate(
            [
                (dyn.FrameType.Operational, pin.FrameType.OP_FRAME),
                (dyn.FrameType.Joint, pin.FrameType.JOINT),
                (dyn.FrameType.Fixed, pin.FrameType.FIXED_JOINT),
                (dyn.FrameType.Body, pin.FrameType.BODY),
                (dyn.FrameType.Sensor, pin.FrameType.SENSOR),
            ]
        ):
            pin_placement = pin.SE3.Random()
            dyn_placement = dyn.SE3(
                rotation=pin_placement.rotation, translation=pin_placement.translation
            )
            dyn_frame_type, pin_frame_type = types

            dyn_frame = dyn.Frame(
                f"frame{i}",
                0,
                i,
                dyn_placement,
                dyn_frame_type,
                dyn.Inertia.zeros(),
            )
            pin_frame = pin.Frame(
                f"frame{i}",
                0,
                i,
                pin_placement,
                pin_frame_type,
                pin.Inertia.Zero(),
            )

            dyn_model.add_frame(dyn_frame, append_inertia=False)
            pin_model.addFrame(pin_frame, append_inertia=False)

            assert_models_equals(self, dyn_model, pin_model)

        assert_models_equals(self, dyn_model, pin_model)


class TestData(unittest.TestCase):
    def test_empty_model_data(self):
        dyn_model = dyn.Model()
        pin_model = pin.Model()

        dyn_data = dyn.Data(dyn_model)
        pin_data = pin.Data(pin_model)

        _ = dyn_data.oMi  # check pinocchio API compatibility

        assert_datas_equals(self, dyn_data, pin_data)

    def test_model_add_joint(self):
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
