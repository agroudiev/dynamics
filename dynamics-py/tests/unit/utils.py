import unittest
import dynamics as dyn
import pinocchio as pin
import numpy as np


def assert_se3_equals(test_case: unittest.TestCase, dyn_se3: dyn.SE3, pin_se3: pin.SE3):
    test_case.assertTrue(np.linalg.norm(dyn_se3.rotation - pin_se3.rotation) < 1e-15)
    test_case.assertTrue(
        np.linalg.norm(dyn_se3.translation - pin_se3.translation) < 1e-15
    )


def assert_joint_types_equals(
    test_case: unittest.TestCase,
    dyn_joint: dyn.JointModel,
    pin_joint: pin.JointModel,
):
    match dyn_joint.joint_type:
        case dyn.JointType.Fixed:
            test_case.assertTrue(False)
        case dyn.JointType.Revolute:
            test_case.assertTrue(pin_joint.shortname().startswith("JointModelR"))


def assert_joint_models_equals(
    test_case: unittest.TestCase,
    dyn_joint_model: dyn.JointModel,
    pin_joint_model: pin.JointModel,
):
    test_case.assertEqual(dyn_joint_model.nq, pin_joint_model.nq)
    test_case.assertEqual(dyn_joint_model.nv, pin_joint_model.nv)
    assert_joint_types_equals(test_case, dyn_joint_model, pin_joint_model)


def assert_models_equals(
    test_case: unittest.TestCase, dyn_model: dyn.Model, pin_model: pin.Model
):
    test_case.assertEqual(dyn_model.nq, pin_model.nq)
    test_case.assertEqual(dyn_model.nv, pin_model.nv)
    test_case.assertEqual(dyn_model.name, pin_model.name)
    test_case.assertTrue((dyn_model.gravity == pin_model.gravity.linear).all())
    test_case.assertEqual(dyn_model.njoints, pin_model.njoints)

    for i in range(1, dyn_model.njoints):  # skip the universe joint
        test_case.assertEqual(dyn_model.joint_names[i], pin_model.names[i])
        test_case.assertEqual(dyn_model.joint_parents[i], pin_model.parents[i])
        assert_se3_equals(
            test_case,
            dyn_model.joint_placements[i],
            pin_model.jointPlacements[i],
        )
        assert_joint_models_equals(
            test_case,
            dyn_model.joint_models[i],
            pin_model.joints[i],
        )


def assert_datas_equals(
    test_case: unittest.TestCase, dyn_data: dyn.Data, pin_data: pin.Data
):
    pass
