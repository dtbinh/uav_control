from .. import cython_control
print('testing cython controller')
import numpy as np


J = np.diag([9.773e-3, 9.773e-3, 1.749e-2])
m = 1.7
gains = np.array([6.5,4.0,0.01,0.3,0.1,0.01],dtype=np.double)
controller = cython_control.c_control(m,J.flatten(),gains)
def test_initialization(benchmark):
    a = np.array([0,0,0],np.double)
    R = np.eye(3).flatten()
    Rc_2dot = np.eye(3).flatten()
    xc = np.zeros((8,3))
    xc[5,:] = [1,0,0]
    xc = xc.flatten()
    b = a.copy()
    moment = np.zeros(4)
    benchmark(controller.position_control,a,a,R,a,xc,moment)
    print(Rc_2dot)
    print('after')
    print(moment)
    assert (a == b).all

def test_kx():
    print(controller.get_kx())

def test_get_Rc():
    Rc_py = np.zeros(9)
    controller.get_Rc(Rc_py)
    print()
    print(np.reshape(Rc_py,(3,3)))
def test_get_Rc_dot():
    Rc_py = np.zeros(9)
    controller.get_Rc_dot(Rc_py)
    print()
    print(np.reshape(Rc_py,(3,3)))
def test_get_Rc_2dot():
    Rc_py = np.zeros(9)
    controller.get_Rc_2dot(Rc_py)
    print()
    print(np.reshape(Rc_py,(3,3)))
