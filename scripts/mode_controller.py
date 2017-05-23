#!/home/kuya/anaconda3/bin/python
from __future__ import print_function, division

# implement mode switching of the controller when the vicon signal is not updating
from controller import *
print('system starting')

# execute only if run as a script
J = np.diag([0.0820, 0.0845, 0.1377])
e3 = np.array([0.,0.,1.])
uav_t = UAV(J, e3)
t_max = 12
N = 100*t_max + 1
t = np.linspace(0,t_max,N)
xd = np.array([0.,0.,0.])
# Initial Conditions
R0 = [[1., 0., 0.],
  [0., -0.9995, -0.0314],
  [0., 0.0314, -0.9995]] # initial rotation
R0 = np.eye(3)
W0 = [0.,0.,0.];   # initial angular velocity
x0 = [0.,0.,0.];  # initial position (altitude?0)
v0 = [0.,0.,0.];   # initial velocity
R0v = np.array(R0).flatten().T
y0 = np.concatenate((R0v, W0,x0,v0))

R =
uav_t.attitude_control(
