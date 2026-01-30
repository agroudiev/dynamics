import unittest
import numpy as np

import dynamics as dyn
import dynamics.collider as collider  # type: ignore

import pinocchio as pin
import coal


def set_ros_package_path(package: str):
    """Set ROS_PACKAGE_PATH to find the given package."""
    import os

    os.environ["ROS_PACKAGE_PATH"] = (
        "examples/descriptions/"
        + package
        + ":"
        + os.environ.get("ROS_PACKAGE_PATH", "")
    )


def assert_se3_equals(test_case: unittest.TestCase, dyn_se3: dyn.SE3, pin_se3: pin.SE3):
    test_case.assertTrue(np.linalg.norm(dyn_se3.rotation - pin_se3.rotation) < 1e-14)
    test_case.assertTrue(
        np.linalg.norm(dyn_se3.translation - pin_se3.translation) < 1e-14
    )


def assert_joint_types_equals(
    test_case: unittest.TestCase,
    dyn_joint: dyn.JointModel,
    pin_joint: pin.JointModel,
):
    match str(dyn_joint.joint_type):
        case "JointType.Continuous":
            test_case.assertTrue(
                pin_joint.shortname().startswith("JointModelRU")
                or pin_joint.shortname() == ("JointModelRevoluteUnboundedUnaligned")
            )
        case "JointType.Fixed":
            test_case.fail("Pinocchio does not have a Fixed joint model")
        case "JointType.Prismatic":
            test_case.assertTrue(
                pin_joint.shortname()
                in [
                    "JointModelPX",
                    "JointModelPY",
                    "JointModelPZ",
                    "JointModelPrismaticUnaligned",
                ]
            )
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
        np.linalg.norm(dyn_inertia.inertia - pin_inertia.inertia) < 1e-14
    )


def assert_frames_equals(
    test_case: unittest.TestCase, dyn_frame: dyn.Frame, pin_frame: pin.Frame
):
    test_case.assertEqual(
        dyn_frame.name,
        pin_frame.name if pin_frame.name != "universe" else "__WORLD_FRAME__",
    )
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
    for i in range(dyn_model.nframes):
        dyn_frame = dyn_model.frames[i]
        pin_frame = pin_model.frames[i]
        assert_frames_equals(test_case, dyn_frame, pin_frame)

    # Check inertias
    test_case.assertEqual(len(dyn_model.inertias), len(pin_model.inertias))
    for i in range(len(dyn_model.inertias)):
        dyn_inertia = dyn_model.inertias[i]
        pin_inertia = pin_model.inertias[i]
        assert_inertias_equals(test_case, dyn_inertia, pin_inertia)


def assert_shapes_equals(
    test_case: unittest.TestCase,
    dyn_shape: collider.Shape,
    pin_shape: pin.CollisionGeometry,
):
    match str(dyn_shape.shape_type):
        case "ShapeType.Capsule":
            test_case.assertEqual(type(pin_shape), coal.coal_pywrap.Capsule)
            test_case.assertTrue(
                np.linalg.norm(dyn_shape.radius - pin_shape.radius) < 1e-7
            )
            test_case.assertTrue(
                np.linalg.norm(dyn_shape.half_length - pin_shape.halfLength) < 1e-7
            )
        case "ShapeType.Cone":
            test_case.assertEqual(type(pin_shape), coal.coal_pywrap.Cone)
            test_case.assertTrue(
                np.linalg.norm(dyn_shape.radius - pin_shape.radius) < 1e-7
            )
            test_case.assertTrue(
                np.linalg.norm(dyn_shape.half_length - pin_shape.halfLength) < 1e-7
            )
        case "ShapeType.Cuboid":
            test_case.assertEqual(type(pin_shape), coal.coal_pywrap.Box)
            test_case.assertTrue(
                np.linalg.norm(dyn_shape.half_extents - pin_shape.halfSide) < 1e-7
            )
        case "ShapeType.Cylinder":
            test_case.assertEqual(type(pin_shape), coal.coal_pywrap.Cylinder)
            test_case.assertTrue(
                np.linalg.norm(dyn_shape.radius - pin_shape.radius) < 1e-7
            )
            test_case.assertTrue(
                np.linalg.norm(dyn_shape.half_length - pin_shape.halfLength) < 1e-7
            )
        case "ShapeType.Sphere":
            test_case.assertEqual(type(pin_shape), coal.coal_pywrap.Sphere)
            test_case.assertTrue(
                np.linalg.norm(dyn_shape.radius - pin_shape.radius) < 1e-7
            )
        case "ShapeType.Mesh":
            pass  # TODO: implement mesh comparison
        case _:
            test_case.fail(f"Unknown shape type '{dyn_shape.shape_type}'")


def assert_geometry_objects_equals(
    test_case: unittest.TestCase,
    dyn_geom: dyn.GeometryObject,
    pin_geom: pin.GeometryObject,
):
    test_case.assertEqual(dyn_geom.name, pin_geom.name)
    test_case.assertEqual(dyn_geom.parent_joint, pin_geom.parentJoint)
    test_case.assertEqual(dyn_geom.parent_frame, pin_geom.parentFrame)
    assert_se3_equals(test_case, dyn_geom.placement, pin_geom.placement)
    assert_shapes_equals(test_case, dyn_geom.geometry, pin_geom.geometry)
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


def assert_joint_datas_equals(
    test_case: unittest.TestCase,
    dyn_joint_data: dyn.JointData,
    pin_joint_data: pin.JointData,
):
    # Check the joint configuration vector
    test_case.assertTrue(
        np.linalg.norm(dyn_joint_data.joint_q.to_numpy() - pin_joint_data.joint_q)
        < 1e-15,
    )

    # Check the joint velocity vector
    # test_case.assertTrue(
    #     np.linalg.norm(dyn_joint_data.joint_v.to_numpy() - pin_joint_data.joint_v)
    #     < 1e-15,
    # )
    # disabled since pinocchio is not consistent for unaligned joints

    # Check the joint placement
    assert_se3_equals(
        test_case,
        dyn_joint_data.joint_placement,
        pin_joint_data.M,
    )

    # Check joint velocity
    test_case.assertTrue(
        np.linalg.norm(dyn_joint_data.v.to_numpy() - pin_joint_data.v) < 1e-14,
    )

    # Check joint acceleration
    # TODO


def assert_datas_equals(
    test_case: unittest.TestCase, dyn_data: dyn.Data, pin_data: pin.Data
):
    # Check joint placements
    dyn_placements = dyn_data.joint_placements  # .oMi also works
    pin_placements = pin_data.oMi
    test_case.assertEqual(len(dyn_placements), len(pin_placements))
    for i in range(len(dyn_placements)):
        assert_se3_equals(test_case, dyn_placements[i], pin_placements[i])

    # Check frame placements
    dyn_frame_placements = dyn_data.frame_placements  # .oMf also works
    pin_frame_placements = pin_data.oMf
    test_case.assertEqual(len(dyn_frame_placements), len(pin_frame_placements))
    for i in range(len(dyn_frame_placements)):
        assert_se3_equals(test_case, dyn_frame_placements[i], pin_frame_placements[i])

    # Check joint data
    for i in range(len(dyn_data.joint_placements)):
        assert_joint_datas_equals(test_case, dyn_data.joints[i], pin_data.joints[i])

    # Check velocities
    test_case.assertEqual(len(dyn_data.v), len(pin_data.v))
    for i in range(len(dyn_data.v)):
        test_case.assertTrue(
            np.linalg.norm(dyn_data.v[i].to_numpy() - pin_data.v[i]) < 1e-14,
            f"{dyn_data.v[i].to_numpy()}\n{pin_data.v[i]}",
        )

    # Check accelerations
    test_case.assertEqual(len(dyn_data.a), len(pin_data.a))
    for i in range(len(dyn_data.a)):
        test_case.assertTrue(
            np.linalg.norm(dyn_data.a[i].to_numpy() - pin_data.a[i]) < 1e-14
        )


EXAMPLE_ROBOT_DATA_URDFS = [
    "a1_description/urdf/a1.urdf",
    #
    "alex_description/urdf/alex_nub_hands.urdf",
    "alex_description/urdf/alex_psyonic_hands.urdf",
    "alex_description/urdf/alex_sake_hands.urdf",
    #
    "alexander_description/urdf/alexander_v1.lowerBodyOnly.urdf",
    #
    "allegro_hand_description/urdf/allegro_left_hand.urdf",
    "allegro_hand_description/urdf/allegro_right_hand.urdf",
    #
    "anymal_b_simple_description/robots/anymal-kinova.urdf",
    "anymal_b_simple_description/robots/anymal.urdf",
    #
    "anymal_c_simple_description/urdf/anymal.urdf",
    #
    "asr_twodof_description/urdf/TwoDofs.urdf",
    #
    "b1_description/urdf/b1-z1.urdf",
    "b1_description/urdf/b1.urdf",
    #
    "baxter_description/urdf/baxter.urdf",
    #
    "bluevolta_description/urdf/bluevolta_bravo7_gripper.urdf",
    "bluevolta_description/urdf/bluevolta_bravo7_no_ee.urdf",
    "bluevolta_description/urdf/bluevolta.urdf",
    #
    "borinot_description/urdf/borinot_flying_arm_2.urdf",
    #
    "bravo7_description/urdf/bravo7_gripper.urdf",
    "bravo7_description/urdf/bravo7_no_ee.urdf",
    #
    "centauro_description/urdf/centauro.urdf",
    #
    "double_pendulum_description/urdf/double_pendulum_continuous.urdf",
    "double_pendulum_description/urdf/double_pendulum_simple.urdf",
    "double_pendulum_description/urdf/double_pendulum.urdf",
    #
    "falcon_description/urdf/falcon_bravo7_gripper.urdf",
    "falcon_description/urdf/falcon_bravo7_no_ee.urdf",
    # "falcon_description/urdf/falcon.urdf", # incorrect URDF
    #
    "finger_edu_description/robots/finger_edu.urdf",
    #
    "g1_description/urdf/g1_29dof_rev_1_0.urdf",
    "g1_description/urdf/g1_29dof_with_hand_rev_1_0.urdf",
    #
    "go1_description/urdf/go1.urdf",
    #
    "go2_description/urdf/go2.urdf",
    #
    "hector_description/robots/quadrotor_base.urdf",
    #
    "hextilt_description/urdf/hextilt_flying_arm_5.urdf",
    #
    "human_description/robots/human.urdf",
    #
    "hyq_description/robots/hyq_no_sensors.urdf",
    #
    "icub_description/robots/icub_reduced.urdf",
    "icub_description/robots/icub.urdf",
    #
    "iris_description/robots/iris_simple.urdf",
    "iris_description/robots/iris.urdf",
    #
    "kinova_description/robots/kinova.urdf",
    #
    "laikago_description/urdf/laikago.urdf",
    #
    "panda_description/urdf/panda_collision.urdf",
    "panda_description/urdf/panda.urdf",
    #
    "pr2_description/urdf/pr2.urdf",
    #
    "quadruped_description/urdf/quadruped.urdf",
    #
    # "romeo_description/urdf/romeo_laas_small.urdf", # local path issue
    "romeo_description/urdf/romeo_small.urdf",
    "romeo_description/urdf/romeo.urdf",
    #
    "simple_humanoid_description/urdf/simple_humanoid_classical.urdf",
    # TODO: collision_checking nodes
    # "simple_humanoid_description/urdf/simple_humanoid.urdf",
    #
    "so_arm_description/urdf/so100.urdf",
    "so_arm_description/urdf/so101.urdf",
    #
    "solo_description/robots/solo.urdf",
    "solo_description/robots/solo12.urdf",
    #
    "talos_data/robots/talos_full_v2_box.urdf",
    "talos_data/robots/talos_full_v2.urdf",
    "talos_data/robots/talos_left_arm.urdf",
    "talos_data/robots/talos_reduced_box.urdf",
    "talos_data/robots/talos_reduced_corrected.urdf",
    "talos_data/robots/talos_reduced.urdf",
    #
    "tiago_description/robots/tiago_dual.urdf",
    "tiago_description/robots/tiago_no_hand.urdf",
    "tiago_description/robots/tiago.urdf",
    #
    "tiago_pro_description/robots/tiago_pro.urdf",
    #
    "ur_description/urdf/ur3_gripper.urdf",
    "ur_description/urdf/ur3_joint_limited_robot.urdf",
    "ur_description/urdf/ur3_robot.urdf",
    "ur_description/urdf/ur5_gripper.urdf",
    "ur_description/urdf/ur5_joint_limited_robot.urdf",
    "ur_description/urdf/ur5_robot.urdf",
    "ur_description/urdf/ur10_joint_limited_robot.urdf",
    "ur_description/urdf/ur10_robot.urdf",
    #
    "xarm_description/urdf/xarm7.urdf",
    #
    "z1_description/urdf/z1.urdf",
]
