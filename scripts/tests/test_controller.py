#!/usr/bin/env python
from .. import controller
import numpy as np
import pdb

J = np.eye(3)
e3 = np.array([0,0,1])


class TestController():
    test_ctrl = controller.Controller(J, e3)

    def test_e3(self):
        assert (self.test_ctrl.e3 == e3).all()

    def test_J_size(self):
        assert self.test_ctrl.J.shape == (3,3)

    def test_mass_range(self):
        assert (self.test_ctrl.m > 0 and self.test_ctrl.m < 5)

    def test_ex(self):
        assert self.test_ctrl.ex is None

    def test_pos_controller(self):
        pass
