from .. import cython_control
print('testing cython controller')
import numpy as np


J = np.diag([9.773e-3, 9.773e-3, 1.749e-2])
m = 1.7
dt = 0.01
gains = np.array([6.5,4.0,0.01,0.3,0.1,0.01],dtype=np.double)
controller = cython_control.c_control(m,dt,J.flatten(),gains)
vec3 = np.ones(3)

def test_init(benchmark):
    a = np.array([0,0,0],np.double)
    R = np.eye(3).flatten()
    Rc_2dot = np.eye(3).flatten()
    xc = np.zeros((8,3))
    xc[0,:] = [-1,0,0]
    xc[5,:] = [1,0,0]
    xc = xc.flatten()
    b = a.copy()
    moment = np.zeros(4)
    benchmark(controller.position_control,a,a,R,a,xc,moment)
    print(Rc_2dot)
    print('after')
    print(moment)
    assert (a == b).all

def test_get_b1d():
    controller.get_b1d(vec3)
    print()
    print(vec3)
    assert (vec3 == np.array([1,0,0])).all()
def test_get_e3():
    controller.get_e3(vec3)
    print()
    print(vec3)

def test_get_ex():
    controller.get_ex(vec3)
    print()
    print(vec3)

def test_get_ev():
    controller.get_ev(vec3)
    print()
    print(vec3)

def test_get_eR():
    controller.get_eR(vec3)
    print()
    print(vec3)

def test_get_Wc():
    controller.get_Wc(vec3)
    print()
    print(vec3)

def test_get_Wc_dot():
    controller.get_Wc_dot(vec3)
    print()
    print(vec3)

def test_get_eiR():
    controller.get_eiR(vec3)
    print()
    print(vec3)

def test_get_eW():
    controller.get_eW(vec3)
    print()
    print(vec3)

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

