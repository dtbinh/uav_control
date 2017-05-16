import pyximport; pyximport.install()
# import controller here
from sim_controller import UAV, hat
import numpy as np
from scipy.integrate import odeint
from scipy.integrate import ode
import numpy.linalg as la
import pdb
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation
import sys



def dydt_pos(t, X, uav_t):
    R = np.reshape(X[6:15],(3,3));  # rotation from body to inertial
    W = X[15:];   # angular rate
    x = X[:3];  # position
    v = X[3:6];    # velocity

    xd = np.array([0, 0, 0])
    xd_dot = np.array([0, 0, 0])
    xd_ddot = np.array([0, 0, 0])
    xd_dddot = np.array([0, 0, 0])
    xd_ddddot = np.array([0, 0, 0])
    b1d = np.array([1., 0., 0.])
    b1d_dot=np.array([0., 0., 0.])
    b1d_ddot=np.array([0., 0., 0.])
    Rd = np.eye(3)
    Wd = np.array([0.,0.,0.])
    Wd_dot = np.array([0.,0.,0.])
    f = np.array([0,0,0])
    M = np.array([0,0,0])

    xd = np.array([0.5*np.sin(t), 0, t])
    xd_dot = np.array([0.5 , 0, 1])
    b1d = np.array([1., 0.,0.])
    d_in = (xd, xd_dot, xd_ddot, xd_dddot, xd_ddddot,
            b1d, b1d_dot, b1d_ddot, Rd, Wd, Wd_dot)
    (f, M) = uav_t.position_control(t, R, W, x, v, d_in)

    R_dot = np.dot(R,hat(W))
    W_dot = np.dot(la.inv(uav_t.J), M - np.cross(W, np.dot(uav_t.J, W)))
    x_dot = v
    v_dot = uav_t.g*uav_t.e3 - f*R.dot(uav_t.e3)/uav_t.m
    X_dot = np.concatenate((x_dot, v_dot, R_dot.flatten(), W_dot))
    uav_t.xd = xd
    uav_t.xd_dot = xd_dot
    uav_t.command = np.insert(M,0,f)
    return X_dot



ukf_flag = False
anim_flag = True
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
y0 = np.concatenate((x0, v0, R0v, W0))

# sim = odeint(uav_t.dydt,y0,t)

solver = ode(dydt_pos)
solver.set_integrator('dopri5').set_initial_value(y0, 0)
dt = 1./100
sim = []
xd = []
xd_dot = []
command_val = []
while solver.successful() and solver.t < t_max:
    solver.set_f_params(uav_t)
    solver.integrate(solver.t+dt)
    sim.append(solver.y)
    xd.append(uav_t.xd)
    xd_dot.append(uav_t.xd_dot)
    command_val.append(uav_t.command)

sim = np.array(sim)
xd = np.array(xd)
xd_dot = np.array(xd_dot)
# def animate(i):
#fig = plt.figure()
#ax = fig.gca(projection = '3d')
#ax.set_aspect('equal')
#ax.plot(sim[:,0],sim[:,1],sim[:,2])
#ax.set_xlim(0, 10)
#ax.set_ylim(-5, 5)
#ax.set_zlim(-5, 5)
#plt.show()
if anim_flag:
    f, ax = plt.subplots(3,2)
    ax[0][0].plot(xd[:,0])
    ax[0][0].plot(sim[:,0])
    ax[0][1].plot(xd_dot[:,0])
    ax[0][1].plot(sim[:,3])

    ax[1][0].plot(xd[:,1])
    ax[1][1].plot(xd_dot[:,1])
    ax[1][0].plot(sim[:,1])
    ax[1][1].plot(sim[:,4])

    ax[2][0].plot(xd[:,2])
    ax[2][1].plot(xd_dot[:,2])
    ax[2][0].plot(sim[:,2])
    ax[2][1].plot(sim[:,5])
    plt.show()
    pass
