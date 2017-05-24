#!/usr/bin/env python
import unittest
import timeit
from .. import controller
import numpy as np
import pdb
import time
J = np.eye(3)
e3 = np.array([0,0,1])


test_ctrl = controller.Controller(J, e3)
force = 0
def test_e3():
    assert (test_ctrl.e3 == e3).all()

def test_J_size():
    assert test_ctrl.J.shape == (3,3)

def test_mass_range():
    assert (test_ctrl.m > 0 and test_ctrl.m < 5)

def test_ex():
    assert test_ctrl.ex is None

def test_gravity():
    np.testing.assert_array_almost_equal(test_ctrl.g, 9.81)

def test_pos_controller(benchmark):
    xc = xcdot = xc2dot = xc3dot = xc4dot = np.zeros(3)
    b1d = [1,0,0]
    b1d_dot = b1d_2dot = np.zeros(3)
    Rc = np.eye(3)
    Wc = Wc_dot = np.zeros(3)
    force, M = benchmark(test_ctrl.position_control,Rc,Wc,xc,xcdot,(xc,xcdot,xc2dot,xc3dot,xc4dot,b1d,b1d_dot,b1d_2dot,Rc,Wc,Wc_dot))
    assert force >= 0
    np.testing.assert_almost_equal(np.mean(M),0)

