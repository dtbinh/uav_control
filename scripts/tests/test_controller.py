#!/usr/bin/env python
import unittest
import timeit
from .. import controller
import numpy as np
import pdb
import time
J = np.eye(3)
e3 = np.array([0,0,1])
from .. import cython_control

test_ctrl = controller.Controller(J, e3)
c_test_ctrl = cython_control.c_control(1.1 ,J.flatten(), np.array([1,1,1,1,1,1,1],np.double))
force = 1
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

def test_pos_c_ctrl(benchmark):
    xc = xcdot = xc2dot = xc3dot = xc4dot = np.zeros(3)
    b1d = [1,0,0]
    b1d_dot = b1d_2dot = np.zeros(3)
    Rc = np.eye(3)
    Rc_2dot = np.zeros(9)
    command = np.zeros(4)
    Wc = Wc_dot = np.zeros(3)
    xc_all = np.array([xc,xcdot,xc2dot,xc3dot,xc4dot,b1d,b1d_dot,b1d_2dot])
    benchmark(c_test_ctrl.position_control, xc,xc,Rc.flatten(), xc, xc,Rc_2dot,command)


