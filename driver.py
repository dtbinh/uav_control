# Driver script to run and test the UAV class
import numpy as np
import controller 
import matplotlib.pyplot as plt

from scipy.integrate import ode, odeint
# execute only if run as a script
J = np.diag([0.0820, 0.0845, 0.1377])
e3 = np.array([0.,0.,1.])
uav_t = controller.UAV(J, e3)
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

# sim = odeint(uav_t.dydt,y0,t)

# solver = ode(uav_t.dydt)
# solver.set_integrator('dopri5').set_initial_value(y0, 0)
# dt = 1./100
# sim = []
# while solver.successful() and solver.t < t_max:
#     solver.integrate(solver.t+dt)
# sim.append(solver.y)
# 
# sim = np.array(sim)
# 
# # fig, ax = plt.subplots()
# fig = plt.figure()
# # ax = p3.Axes3D(fig)
# xs = sim[:,-6]
# ys = sim[:,-5]
# zs = sim[:,-4]

