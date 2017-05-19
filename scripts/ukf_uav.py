#!/bin/bash/python
from __future__ import absolute_import, division, print_function, unicode_literals
import numpy as np
# from numba import jit, int16
from numpy.linalg import cholesky, inv, det
import matplotlib.pyplot as plt
import pdb
import matplotlib.pyplot as plt


class UnscentedKalmanFilter(object):
    """Filtering implementation for state estimation
    This module estimates the state of the UAV from given sensor meaurements
    using unscented Kalman filter.
    state:
        x: position, velocity
        R: attitude, angular velocity
        power: current, voltage, RPM
    state vector:
        state: [x,x_dot,R,W]
    """
    def __init__(self, dim_x=1, dim_z=1, dt=0.1, hx=None, fx=None):
        self._dt = dt
        self._dim_x = dim_x
        self._dim_z = dim_z
        self.hx = hx
        self.fx = fx
        self.x = np.zeros(dim_x)
        self.P = np.eye(dim_x)
        self.Q = np.eye(dim_x)
        self.R = np.eye(dim_z)
        self.m = 1.5
        self.g = 9.8
        self.J = np.eye(3)
        self.R = np.eye(3)
        self.e3 = np.array([0,0,1])
        self.Rb = np.eye(3)
        print('Unscented Kalman Filter initialized')

    def sigmaPoints(self, x, P, c):
        """Simg points computation"""
        A = c*cholesky(P).T
        Y = np.tile(x,((A.shape)[0],1)).T
        return np.vstack((x, Y+A, Y-A)).T

    def dss(self, x, u, dt=None):
        if dt == None:
            dt = self._dt
        """State space UAV dynamics"""
        A = np.zeros((12,12))
        for i in range(3):
            A[i][i], A[i][i+3] = 0, dt
        for i in range(6,9):
            A[i][i], A[i][i+3] = 0, dt
            # A[i+6][i+6] = 1
        # TODO: Implement controller input term here
        # noting that u = [F, M] remember to bring them to inertial frame
        B = np.zeros((self._dim_x, 6))
        B[5,2] = 1/self.m
        B[-3:,-3:] = self.J
        uf = -u[0]*self.Rb.dot(self.e3)
        uf = np.append( uf, self.Rb.dot(u[1:]))
        #A
        xp = A.dot(x)  #+ self._dt*B.dot(uf)
        return x + A.dot(x) + dt*B.dot(uf)

    def sss(self, x, u = None):
        """Sensor state"""
        n = len(x)
        if u is None:
            n_c = 1
        else:
            n_c = len(u)
        A = np.zeros((n_c, n))
        for i in range(n_c):
            A[i][i+n_c - 1] = 1
        return x

    def f(self, x, u = None):
        """nonlinear state function"""
        # x += self._dt*np.cos(np.pi*u)
        return x

    def h(self, x, u = None):
        """nonlinear sensor function"""
        return x

    def ut(self, func, x, wm, wc, n_f, Q, u = None):
        """unscented transform
        args:
            f: nonlinear map
        output:
            mean
            sampling points
            covariance
            deviations
        """
        n, m = x.shape
        X = np.zeros((n_f,m))
        mu = np.zeros(n_f)
        for i in range(m):
            X[:,i] = func(x[:,i], u)
            mu = mu + wm[i]*X[:,i]
        Xd = X - np.tile(mu,(m,1)).T
        Sigma = Xd.dot(np.diag(wc.T).dot(Xd.T)) + Q
        return (mu, X, Sigma, Xd)
    def unscented_transform(self, sigmas, Wm, Wc,):
        x = np.dot(Wm, sigmas)
        y = simgas - x[np.newaxis,:]
        P = y.T.dot(np.diag(Wc)).dot(y)
        return x,P

    def predict(self, dt = None, UT=None):
        if dt is None:
            dt = self._dt
        sigmas = self.sigmaPoints(self.x, self.P)
        self.x, self.P = self.ut(self.dss, self.X, self.Wm, self.Wc, self.n, self.Q, self.u)

    def update(self, z, R=None, UT=None):
        if z is None:
            return

        zp, Pz = self.unscented_transform( self.Wm, self.Wc, self.n, self.R)
        K = np.dot(Pxz,inv(Pz))


    def ukf(self, x, P, z, Q, R, u, state_transition = None, state_observation = None):
        """UKF
        args:
            x: a priori state estimate
            P: a priori state covariance
            z: current measurement
            Q: process noise
            R: measurement noise
        output:
            x: a posteriori state estimation
            P: a posteriori state covariance
        """
        n = len(x)
        m = len(x)
        alpha = 0.25
        kappa = 50.
        beta = 2.
        lamb = alpha**2*(n+kappa)-n
        c_n = n+lamb
        Wm = np.append(lamb/c_n, 0.5/c_n+np.zeros(2*n))
        Wc = np.copy(Wm)
        Wc[0] +=  (1-alpha**2+beta)
        c_nr=np.sqrt(c_n)
        X = self.sigmaPoints(x,P,c_nr)
        x1, X1, P1, X2 = self.ut(state_transition, X, Wm, Wc, n, Q, u)
        z1,Z1,P2,Z2 = self.ut(state_observation, X1,Wm,Wc, n, R)
        P12=X2.dot(np.diag(Wc).dot(Z2.T))
        try:
            inv(P2)
        except:
            print('non positive def')
            return x, P

        K=P12.dot(inv(P2))
        x=x1+K.dot(z-z1)
        P=P1-K.dot(P12.T)
        return x, P

    # @jit(int16(int16))
#    def test(self, a = 3):
#        x = a
#        return x

if __name__=='__main__':
    # test(4)
    # test.inspect_types()
    # print('test')
    # ukf_t.ukf(x,P, z, Q, R, u)
    Ns = 3 # number of states
    ukf_t = UnscentedKalmanFilter(Ns, Ns, 0.01)
    s = np.zeros(Ns)
    u = np.zeros(Ns)
    q=0.001
    r= 1
    Q = q**2*np.eye(Ns)
    R = r**2
    P = np.eye(Ns)
    x = s+q*(0.5 - np.random.random(Ns))
    N = 200
    t_sim = np.linspace(0,2,num = N)
    xGT = np.zeros((Ns,N))
    xGT[0,:] = np.sin(np.pi*t_sim)
    xGT[1,:] = np.sin(np.pi*t_sim)
    xV = np.zeros((Ns,N))
    sV = np.copy(xV)
    zV = np.zeros((Ns,N))
    for i in range(N):
        x_cur = xGT[:,i]
        z = ukf_t.h(x_cur+r*(0.5-np.random.random(Ns)))
        zV[:,i] = z
        x, P = ukf_t.ukf(x, P, z, Q, R, t_sim[i], state_transition = ukf_t.f, state_observation = ukf_t.h)
        xV[:,i] = x
        #x = ukf_t.f(x,u) + q*np.random.random(Ns)
        pass
    f, (ax0,ax1,ax2) = plt.subplots(3,1)
    ax0.plot(xGT[0],'b')
    ax0.plot(xV[0],'r.-')
    ax0.plot(zV[0],'gx')
    ax1.plot(xGT[1],'b')
    ax1.plot(xV[1],'r.-')
    ax1.plot(zV[1],'gx')
    plt.show()
