import unittest
import dynamics as dyn
import numpy as np
import sys
import os
import warnings
from contextlib import contextmanager
from utils import set_ros_package_path, EXAMPLE_ROBOT_DATA_URDFS
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

    @parameterized.expand(EXAMPLE_ROBOT_DATA_URDFS)
    def test_viz_example_robot_data(self, path):
        set_ros_package_path("example-robot-data")
        robots_dir = "examples/descriptions/example-robot-data/robots/"
        with visualize_cm():
            test_visualizer(
                self,
                robots_dir + path,
            )
