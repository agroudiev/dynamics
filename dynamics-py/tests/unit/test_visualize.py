import unittest
import dynamics as dyn
import numpy as np
import sys
import os
import warnings
from contextlib import contextmanager


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
        q += np.random.randn(model.nq) * 0.1
        viz.display(q)

    viz.clean()


class TestURDF(unittest.TestCase):
    def test_viz_myfirst(self):
        with visualize_cm():
            test_visualizer(self, "examples/descriptions/myfirst.urdf")

    def test_build_multipleshapes(self):
        with visualize_cm():
            test_visualizer(self, "examples/descriptions/multipleshapes.urdf")

    def test_build_double_pendulum_simple(self):
        with visualize_cm():
            test_visualizer(self, "examples/descriptions/double_pendulum_simple.urdf")

    def test_build_materials(self):
        with visualize_cm():
            test_visualizer(self, "examples/descriptions/materials.urdf")

    def test_build_origins(self):
        with visualize_cm():
            test_visualizer(self, "examples/descriptions/origins.urdf")

    def test_build_visuals(self):
        with visualize_cm():
            test_visualizer(self, "examples/descriptions/visuals.urdf")

    def test_build_ur5(self):
        with visualize_cm():
            test_visualizer(
                self,
                "./examples/descriptions/ur5/ur5_robot.urdf",
                "./examples/descriptions/ur5",
            )
