import unittest
import dynamics as dyn
import pinocchio as pin
from utils import (
    assert_models_equals,
    assert_geometry_models_equals,
    assert_datas_equals,
    set_ros_package_path,
    EXAMPLE_ROBOT_DATA_URDFS,
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

    @parameterized.expand(EXAMPLE_ROBOT_DATA_URDFS)
    def test_build_example_robot_data(self, path):
        set_ros_package_path("example-robot-data")
        robots_dir = "examples/descriptions/example-robot-data/robots/"
        compare_urdf_construction(
            self,
            robots_dir + path,
        )

    def test_build_upkie(self):
        set_ros_package_path("upkie_description")
        compare_urdf_construction(
            self,
            "examples/descriptions/upkie_description/urdf/upkie.urdf",
        )
