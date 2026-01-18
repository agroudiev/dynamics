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
    match str(dyn_joint.joint_type):
        case "JointType.Fixed":
            test_case.fail("Pinocchio does not have a Fixed joint model")
        case "JointType.Revolute":
            test_case.assertTrue(pin_joint.shortname().startswith("JointModelR"))
        case _:
            test_case.fail(f"Unknown joint type '{dyn_joint.joint_type}'")


def assert_joint_models_equals(
    test_case: unittest.TestCase,
    dyn_joint_model: dyn.JointModel,
    pin_joint_model: pin.JointModel,
):
    test_case.assertEqual(dyn_joint_model.nq, pin_joint_model.nq)
    test_case.assertEqual(dyn_joint_model.nv, pin_joint_model.nv)
    assert_joint_types_equals(test_case, dyn_joint_model, pin_joint_model)


def assert_inertias_equals(
    test_case: unittest.TestCase, dyn_inertia: dyn.Inertia, pin_inertia: pin.Inertia
):
    test_case.assertAlmostEqual(dyn_inertia.mass, pin_inertia.mass)
    test_case.assertTrue(np.linalg.norm(dyn_inertia.com - pin_inertia.lever) < 1e-15)
    test_case.assertTrue(
        np.linalg.norm(dyn_inertia.inertia - pin_inertia.inertia) < 1e-15
    )


def assert_frames_equals(
    test_case: unittest.TestCase, dyn_frame: dyn.Frame, pin_frame: pin.Frame
):
    test_case.assertEqual(dyn_frame.name, pin_frame.name)
    test_case.assertEqual(dyn_frame.parent_joint, pin_frame.parentJoint)
    test_case.assertEqual(dyn_frame.parent_frame, pin_frame.parentFrame)
    match str(dyn_frame.frame_type):
        case "FrameType.Operational":
            test_case.assertEqual(pin_frame.type, pin.FrameType.OP_FRAME)
        case "FrameType.Joint":
            test_case.assertEqual(pin_frame.type, pin.FrameType.JOINT)
        case "FrameType.Fixed":
            test_case.assertEqual(pin_frame.type, pin.FrameType.FIXED_JOINT)
        case "FrameType.Body":
            test_case.assertEqual(pin_frame.type, pin.FrameType.BODY)
        case "FrameType.Sensor":
            test_case.assertEqual(pin_frame.type, pin.FrameType.SENSOR)
        case _:
            test_case.fail(f"Unknown frame type '{dyn_frame.frame_type}'")
    assert_se3_equals(test_case, dyn_frame.placement, pin_frame.placement)
    assert_inertias_equals(test_case, dyn_frame.inertia, pin_frame.inertia)


def assert_models_equals(
    test_case: unittest.TestCase, dyn_model: dyn.Model, pin_model: pin.Model
):
    test_case.assertEqual(dyn_model.nq, pin_model.nq)
    test_case.assertEqual(dyn_model.nv, pin_model.nv)
    test_case.assertEqual(dyn_model.name, pin_model.name)
    test_case.assertTrue((dyn_model.gravity == pin_model.gravity.linear).all())

    # Check joints
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

    # Check frames
    test_case.assertEqual(dyn_model.nframes, pin_model.nframes)
    for i in range(1, dyn_model.nframes):  # skip the world frame
        dyn_frame = dyn_model.frames[i]
        pin_frame = pin_model.frames[i]
        assert_frames_equals(test_case, dyn_frame, pin_frame)

    # Check inertias
    test_case.assertEqual(len(dyn_model.inertias), len(pin_model.inertias))
    for i in range(len(dyn_model.inertias)):
        dyn_inertia = dyn_model.inertias[i]
        pin_inertia = pin_model.inertias[i]
        assert_inertias_equals(test_case, dyn_inertia, pin_inertia)


def assert_geometry_objects_equals(
    test_case: unittest.TestCase,
    dyn_geom: dyn.GeometryObject,
    pin_geom: pin.GeometryObject,
):
    test_case.assertEqual(dyn_geom.name, pin_geom.name)
    test_case.assertEqual(dyn_geom.parent_joint, pin_geom.parentJoint)
    test_case.assertEqual(dyn_geom.parent_frame, pin_geom.parentFrame)
    assert_se3_equals(test_case, dyn_geom.placement, pin_geom.placement)
    # assert_geometries_equals(test_case, dyn_geom.geometry, pin_geom.geometry)
    test_case.assertEqual(dyn_geom.disable_collision, pin_geom.disableCollision)
    test_case.assertTrue(
        np.linalg.norm(dyn_geom.mesh_color - pin_geom.meshColor) < 1e-5,
    )


def assert_geometry_models_equals(
    test_case: unittest.TestCase,
    dyn_geom_model: dyn.GeometryModel,
    pin_geom_model: pin.GeometryModel,
):
    test_case.assertEqual(dyn_geom_model.ngeoms, pin_geom_model.ngeoms)
    for i in range(dyn_geom_model.ngeoms):
        dyn_geom = dyn_geom_model.geometry_objects[i]
        pin_geom = pin_geom_model.geometryObjects[i]
        assert_geometry_objects_equals(test_case, dyn_geom, pin_geom)


def assert_datas_equals(
    test_case: unittest.TestCase, dyn_data: dyn.Data, pin_data: pin.Data
):
    # Check joint placements
    dyn_placements = dyn_data.joint_placements  # .oMi also works
    pin_placements = pin_data.oMi
    test_case.assertEqual(len(dyn_placements), len(pin_placements))
    for i in range(len(dyn_placements)):
        assert_se3_equals(test_case, dyn_placements[i], pin_placements[i])

    # Check joint data
    # TODO

    # Check velocities
    # TODO

    # Check accelerations
    # TODO
