from .. import cython_control
print('testing cython controller')
import numpy as np


def test_initialization(benchmark):
    J = np.diag([9.773e-3, 9.773e-3, 1.749e-2])
    m = 1.7
    gains = np.array([6.5,4.0,0.01,0.3,0.1,0.01],dtype=np.double)
    controller = cython_control.c_control(m,J.flatten(),gains)
    a = np.array([0,0,0],np.double)
    R = np.eye(3).flatten()
    Rc_2dot = np.eye(3).flatten()
    xc = np.zeros((8,3))
    xc[5,:] = [1,0,0]
    xc = xc.flatten()
    b = a.copy()
    moment = np.zeros(4)
    benchmark(controller.position_control,a,a,R,a,xc,Rc_2dot,moment)
    print(Rc_2dot)
    print('after')
    print(moment)
    assert (a == b).all
