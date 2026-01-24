import unittest
import dynamics as dyn
import pinocchio as pin
from utils import (
    assert_models_equals,
    assert_geometry_models_equals,
    assert_datas_equals,
    set_example_robot_data_path,
)
from parameterized import parameterized


def compare_urdf_construction(test_case, file_path, mesh_dir=None):
    dyn_model, dyn_col_model, dyn_viz_model = dyn.build_models_from_urdf(
        file_path, mesh_dir
    )
    pin_model, pin_col_model, pin_viz_model = pin.buildModelsFromUrdf(
        file_path, mesh_dir
    )
    assert_models_equals(test_case, dyn_model, pin_model)
    assert_geometry_models_equals(test_case, dyn_col_model, pin_col_model)
    assert_geometry_models_equals(test_case, dyn_viz_model, pin_viz_model)

    dyn_data = dyn.Data(dyn_model)
    pin_data = pin.Data(pin_model)
    assert_datas_equals(test_case, dyn_data, pin_data)


class TestURDF(unittest.TestCase):
    def test_build_myfirst(self):
        compare_urdf_construction(self, "examples/descriptions/myfirst.urdf")

    def test_build_multipleshapes(self):
        compare_urdf_construction(self, "examples/descriptions/multipleshapes.urdf")

    def test_build_double_pendulum_simple(self):
        compare_urdf_construction(
            self, "examples/descriptions/double_pendulum_simple.urdf"
        )

    def test_build_materials(self):
        compare_urdf_construction(self, "examples/descriptions/materials.urdf")

    def test_build_origins(self):
        compare_urdf_construction(self, "examples/descriptions/origins.urdf")

    def test_build_visuals(self):
        compare_urdf_construction(self, "examples/descriptions/visuals.urdf")

    def test_build_ur5_classical(self):
        compare_urdf_construction(
            self,
            "./examples/descriptions/ur5/ur5_robot.urdf",
            "./examples/descriptions/ur5",
        )

    @parameterized.expand(
        [
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
            # TODO: prismatic joints
            # "baxter_description/urdf/baxter.urdf",
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
            # TODO: fix
            # "falcon_description/urdf/falcon_bravo7_gripper.urdf",
            # "falcon_description/urdf/falcon_bravo7_no_ee.urdf",
            # "falcon_description/urdf/falcon.urdf",
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
            # TODO: prismatic joints
            # "panda_description/urdf/panda_collision.urdf",
            # "panda_description/urdf/panda.urdf",
            #
            # TODO: prismatic joints
            # "pr2_description/urdf/pr2.urdf",
            #
            "quadruped_description/urdf/quadruped.urdf",
            #
            # TODO: fix
            # "romeo_description/urdf/romeo_laas_small.urdf",
            # "romeo_description/urdf/romeo_small.urdf",
            # "romeo_description/urdf/romeo.urdf",
            #
            "simple_humanoid_description/urdf/simple_humanoid_classical.urdf",
            # TODO: fix
            # "simple_humanoid_description/urdf/simple_humanoid.urdf",
            #
            "so_arm_description/urdf/so100.urdf",
            "so_arm_description/urdf/so101.urdf",
            #
            "solo_description/robots/solo.urdf",
            "solo_description/robots/solo12.urdf",
            #
            # TODO: fix
            # "talos_data/robots/talos_full_v2_box.urdf",
            # "talos_data/robots/talos_full_v2.urdf",
            # "talos_data/robots/talos_left_arm.urdf",
            # "talos_data/robots/talos_reduced_box.urdf",
            # "talos_data/robots/talos_reduced_corrected.urdf",
            # "talos_data/robots/talos_reduced.urdf",
            #
            # TODO: prismatic joints
            # "tiago_description/robots/tiago_dual.urdf",
            # "tiago_description/robots/tiago_no_hand.urdf",
            # "tiago_description/robots/tiago.urdf",
            #
            # TODO: prismatic joints
            # "tiago_pro_description/robots/tiago_pro.urdf",
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
    )
    def test_build_example_robot_data(self, path):
        set_example_robot_data_path()
        robots_dir = "examples/descriptions/example-robot-data/robots/"
        compare_urdf_construction(
            self,
            robots_dir + path,
        )
