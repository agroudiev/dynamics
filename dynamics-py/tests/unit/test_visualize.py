import unittest
import dynamics as dyn
import numpy as np
import sys
import os
import warnings
from contextlib import contextmanager
from utils import set_ros_package_path
from parameterized import parameterized


@contextmanager
def visualize_cm():
    """
    Suppress Meshcat ResourceWarning during tests, and redirect
    stdout to null to avoid URL printouts.
    """
    warnings.simplefilter("ignore", ResourceWarning)
    sys.stdout = open(os.devnull, "w")
    try:
        yield
    finally:
        sys.stdout.close()
        sys.stdout = sys.__stdout__


def test_visualizer(test_case, file_path, mesh_dir=None):
    model, coll_model, viz_model = dyn.build_models_from_urdf(file_path, mesh_dir)

    q = dyn.neutral(model)
    q = q.to_numpy()

    viz = dyn.visualize.MeshcatVisualizer(model, coll_model, viz_model)
    viz.init_viewer(load_model=True)

    for _ in range(10):
        q += np.random.randn(model.nq) * 10.0
        viz.display(q)

    viz.clean()


class TestURDF(unittest.TestCase):
    def test_viz_myfirst(self):
        with visualize_cm():
            test_visualizer(self, "examples/descriptions/myfirst.urdf")

    def test_viz_multipleshapes(self):
        with visualize_cm():
            test_visualizer(self, "examples/descriptions/multipleshapes.urdf")

    def test_viz_double_pendulum_simple(self):
        with visualize_cm():
            test_visualizer(self, "examples/descriptions/double_pendulum_simple.urdf")

    def test_viz_materials(self):
        with visualize_cm():
            test_visualizer(self, "examples/descriptions/materials.urdf")

    def test_viz_origins(self):
        with visualize_cm():
            test_visualizer(self, "examples/descriptions/origins.urdf")

    def test_viz_visuals(self):
        with visualize_cm():
            test_visualizer(self, "examples/descriptions/visuals.urdf")

    def test_viz_ur5_classical(self):
        with visualize_cm():
            test_visualizer(
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
    )
    def test_viz_example_robot_data(self, path):
        set_ros_package_path("example-robot-data")
        robots_dir = "examples/descriptions/example-robot-data/robots/"
        with visualize_cm():
            test_visualizer(
                self,
                robots_dir + path,
            )
