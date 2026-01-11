import unittest
import dynamics as dyn
import pinocchio as pin
from utils import assert_models_equals, assert_datas_equals


class TestModel(unittest.TestCase):
    def test_empty_model(self):
        assert_models_equals(self, dyn.Model(), pin.Model())


class TestData(unittest.TestCase):
    def test_empty_model_data(self):
        dyn_model = dyn.Model()
        pin_model = pin.Model()

        dyn_data = dyn.Data(dyn_model)
        pin_data = pin.Data(pin_model)

        _ = dyn_data.oMf  # check pinocchio API compatibility

        assert_datas_equals(self, dyn_data, pin_data)
