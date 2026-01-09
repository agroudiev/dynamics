import unittest
import dynamics as dyn
import pinocchio as pin
from utils import assert_models_equals


class TestModel(unittest.TestCase):
    def test_empty_model(self):
        assert_models_equals(self, dyn.Model(), pin.Model())
