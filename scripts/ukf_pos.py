import pyximport; pyximport.install()
# import controller here
from sim_controller import UAV, hat, rot_eul
import numpy as np
from scipy.integrate import odeint
from scipy.integrate import ode
import numpy.linalg as la
import pdb
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation
import sys
import ukf_uav

from filterpy.kalman import UKF, JulierSigmaPoints


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

    xd = np.array([np.sin(np.pi*t/6), np.cos(np.pi*t/6)-1, t])
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
anim_flag = False
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


Ns = 12
ukf_test = ukf_uav.UnscentedKalmanFilter(Ns, Ns, 0.01)

q = 1
r = 1
ukf_test.J = J
ukf_test.e3 = e3
Q = q**2*np.eye(Ns)
R = r**2
x = np.zeros(Ns)
P = np.eye(Ns)

x_ukf = []
x_sensor = []
sp = JulierSigmaPoints(Ns,5)
def state_tran(x, dt):
    A = np.eye(12)
    for i in range(3):
        A[i,i+3] = dt
        A[i+6,i+9] = dt
    return np.dot(A,x)
def obs_state(x):
    A = np.zeros((6,12))
    for i in range(3):
        A[i,i+3] = 1
        A[i+3, i + 9] = 1
    return np.dot(A,x)
ukf_filter = UKF.UnscentedKalmanFilter(dim_x=Ns,dim_z=6, dt=0.01, hx = obs_state, fx = state_tran, points = sp)
ukf_filter.Q = Q
ukf_filter.R = R
ukf_filter.P = P
for i, state in enumerate(sim):
    ukf_filter.predict()
    Rot = np.reshape(state[6:15],(-1,9))
    Rot_e = rot_eul(Rot)
    noise = np.zeros(Ns)
    noise[:3] = r*(0.5-np.random.random(3))
    x_obs = np.concatenate((state[:6],np.reshape(Rot_e,(3,)),state[-3:])) # + noise
    x_obs[:3] += r*(0.5 - np.random.random(3))
    x_sensor.append(x_obs)
    ukf_filter.update(obs_state(x_obs))
    x = ukf_filter.x
    x_ukf.append(x)

#sys.exit()
#x = 0.001*(0.5-np.random.random(Ns))
#for i, k in enumerate(sim):
#    Rot = np.reshape(k[6:15],(-1,9))
#    Rot_e = rot_eul(Rot)
#    noise = np.zeros(Ns)
#    noise[:3] = r*(0.5-np.random.random(3))
#    x_obs = np.concatenate((k[:6],np.reshape(Rot_e,(3,)),k[-3:])) + noise
#    z = ukf_test.sss(x_obs)# + r*(0.5-np.random.random(6))
#    x_sensor.append(z)
#    ukf_test.Rb = Rot.reshape((3,3))
#    x, P = ukf_test.ukf( x, P, z, Q, R, command_val[i], state_transition = ukf_test.dss, state_observation = ukf_test.sss)
#    # x_obs = ukf_test.dss(x,command_val[i])# + q*(0.5-np.random.random(Ns))
#    x_ukf.append(x)

x_estimate = np.array(x_ukf)
x_sensor = np.array(x_sensor)
f, (ax0, ax1, ax2) = plt.subplots(3,1)
ax0.plot(sim[:,0],'b--')
ax0.plot(x_sensor[:,0],'g.')
ax0.plot(x_estimate[:,0],'r')
ax1.plot(sim[:,1],'b--')
ax1.plot(x_sensor[:,1],'g.')
ax1.plot(x_estimate[:,1],'r')
ax2.plot(sim[:,2],'b--')
ax2.plot(x_sensor[:,2],'g.')
ax2.plot(x_estimate[:,2],'r')

plt.show()
sys.exit()


fig = plt.figure()
ax = p3.Axes3D(fig)
xs = sim[:,-6]
ys = sim[:,-5]
zs = sim[:,-4]

x_estimate = np.array(x_ukf)
x_sensor = np.array(x_sensor)
ax.plot(xs,ys,zs,'b')
ax.plot(x_estimate[:,0],x_estimate[:,1],x_estimate[:,2],'r')
ax.plot(x_sensor[:,0],x_sensor[:,1],x_sensor[:,2],'g--')
fig.show()
