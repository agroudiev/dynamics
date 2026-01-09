import unittest
import dynamics as dyn
import pinocchio as pin


def compare_urdf_construction(test_case, file_path):
    dyn_model, dyn_geom_model = dyn.build_models_from_urdf(file_path)
    pin_model, pin_col_model, pin_vis_model = pin.buildModelsFromUrdf(file_path)

    test_case.assertEqual(dyn_model.nq, pin_model.nq)
    test_case.assertEqual(dyn_model.nv, pin_model.nv)

    # TODO: compare more properties


class TestURDF(unittest.TestCase):
    def test_build_myfirst(self):
        compare_urdf_construction(self, "examples/descriptions/myfirst.urdf")

    def test_build_double_pendulum_simple(self):
        compare_urdf_construction(
            self, "examples/descriptions/double_pendulum_simple.urdf"
        )

    def test_build_materials(self):
        compare_urdf_construction(self, "examples/descriptions/materials.urdf")

    def test_build_multipleshapes(self):
        compare_urdf_construction(self, "examples/descriptions/multipleshapes.urdf")

    def test_build_origins(self):
        compare_urdf_construction(self, "examples/descriptions/origins.urdf")

    def test_build_visuals(self):
        compare_urdf_construction(self, "examples/descriptions/visuals.urdf")
