from __future__ import print_function, division, absolute_import

from kinematics import attitude as att

import numpy as np
import numpy.linalg as la
import pdb
import sys


class UAV(object):
    """UAV controller
        This module is an implementation of nonlinear geometric controller
        developped by Dr. Taeyoung Lee. The class function with given physical
        parameter of UAV the controller provide position, velocity, and attitude
        nonlinear geometric control for the UAV.

        Example:
            Example from the paper can be simulated using the following:
                $ python example_modes.py
        Keyword argumanets
        J -- Inertial matrix for the UAV
        e3 --

        Todo:
            * separate integration from main.py to example_modes.py
            * hardware interface code must be tested for Nvidia Jetson and
                Odroid (especially I2C communication)
            * trajectory tracking code separation, remove time argument from
                controllers
        .. _github source code:
            https://github.com/fdcl-gwu/UAV_control.git
    """
    def __init__(self, J, e3):
        """Initialization of UAV with known pythical properties"""
        self.m = 4.34
        self.g = 9.81
        self.J = J
        self.e3 = e3
        self.kR = 8.81; # attitude gains
        self.kW = 2.54; # attitude gains
        self.kx = 16.*self.m # position gains
        self.kv = 5.6*self.m # position gains
        print('UAV: initialized')

    def position_control(self, t, R, W, x, v, d_in):
        """Geometric Position Controller

        This ...

        Args:
        Returns:
        """
        (xd, xd_dot, xd_2dot, xd_3dot, xd_4dot, b1d, b1d_dot, b1d_ddot,
                Rd, Wd, Wd_dot) = d_in
        (ex, ev) = position_errors( x, xd, v, xd_dot)

        f = np.dot(self.kx*ex + self.kv*ev + self.m*self.g*self.e3
                - self.m*xd_2dot, R.dot(self.e3) )
        W_hat = hat(W)
        R_dot = R.dot(W_hat)
        x_2dot = self.g*self.e3 - f*R.dot(self.e3)/self.m
        ex_2dot = x_2dot - xd_2dot

        f_dot = (( self.kx*ev + self.kv*ex_2dot
                + self.m*xd_3dot).dot(R.dot(self.e3))
                + ( self.kx*ex + self.kv*ev + self.m*self.g*self.e3
                - self.m*xd_3dot).dot(np.dot(R_dot,self.e3)))

        x_3dot = -1/self.m*( f_dot*R + f*R_dot ).dot(self.e3)
        ex_3dot = x_3dot - xd_3dot

        A = -self.kx*ex - self.kv*ev - self.m*self.g*self.e3 + self.m*xd_2dot
        A_dot = -self.kx*ev - self.kv*ex_2dot + self.m*xd_3dot
        A_2dot = -self.kx*ex_2dot - self.kv*ex_3dot + self.m*xd_4dot

        (Rd, Wd, Wd_dot) = get_Rc(A, A_dot, A_2dot , b1d, b1d_dot, b1d_ddot)

        (eR, eW) = attitude_errors( R, Rd, W, Wd )
        M= (-self.kR*eR - self.kW*eW + np.cross(W, self.J.dot(W))
            - self.J.dot(W_hat.dot(R.T.dot(Rd.dot(Wd)))
            - R.T.dot(Rd.dot(Wd_dot))))
        return (f, M)

    def velocity_control(self, t, R, W, x, v, d_in):
        """Geometric Velocity Controller"""
        (xd, xd_dot, xd_2dot, xd_3dot, xd_4dot,
                b1d, b1d_dot, b1d_ddot, Rd, Wd, Wd_dot) = d_in
        (ex, ev) = position_errors( x, xd, v, xd_dot)

        f = (self.kx*ev + self.m*self.g*self.e3
                - self.m*xd_2dot).dot(R.dot(self.e3))
        W_hat = hat(W)
        R_dot = R.dot(W_hat)
        x_2dot = self.g*self.e3 - f*R.dot(self.e3)/self.m
        ex_2dot = x_2dot - xd_2dot

        f_dot = (( self.kx*ex_2dot - self.m*xd_3dot).dot(R.dot(self.e3))
            + ( self.kx*ev + self.m*self.g*self.e3
                    - self.m*xd_3dot).dot(np.dot(R_dot,self.e3)))

        x_3dot = -1/self.m*( f_dot*R + f*R_dot ).dot(self.e3)
        ex_3dot = x_3dot - xd_3dot

        A = - self.kv*ev - self.m*self.g*e3 + self.m*xd_2dot
        A_dot = - self.kv*ex_2dot + self.m*xd_3dot
        A_2dot = - self.kv*ex_3dot + self.m*xd_4dot

        (Rd, Wd, Wd_dot) = get_Rc(A, A_dot, A_2dot , b1d, b1d_dot, b1d_ddot)
        (eR, eW) = attitude_errors( R, Rd, W, Wd )
        M= (-self.kR*eR - self.kW*eW
            + np.cross(W, self.J.dot(W))
            - self.J.dot(W_hat.dot(R.T.dot(Rd.dot(Wd)))
            - R.T.dot(Rd.dot(Wd_dot))))
        return (f, M)

    def attitude_control(self, t, R, W, x, v, d_in):
        """Geometric Attitude Controller
        Args:
            R (mat): Attitude
            W (mat): Angular velocity
            x:
        Returns:
            f (scalar): force scalar
            M (vector): moment vector
        """
        (xd, xd_dot, xd_ddot, xd_dddot, xd_ddddot,
                b1d, b1d_dot, b1d_ddot, Rd, Wd, Wd_dot) = d_in
        (ex, ev) = position_errors( x, xd, v, xd_dot)
        f = (self.kx*ex + self.kv*v + self.m*self.g*self.e3).dot(R.dot(self.e3))
        W_hat = hat(W)
        (eR, eW) = attitude_errors( R, Rd, W, Wd )
        M= (-self.kR*eR - self.kW*eW + np.cross(W, self.J.dot(W))
            - self.J.dot(W_hat.dot(R.T.dot(Rd.dot(Wd)))
            - R.T.dot(Rd.dot(Wd_dot))))
        return (f, M)

def get_Rc(A, A_dot, A_2dot, b1d, b1d_dot, b1d_ddot):
    # move this as a function
    norm_A = la.norm(A)
    b3c = - A/norm_A
    b3c_dot = - A_dot/norm_A + ( np.dot(A, A_dot)*A )/norm_A**3
    b3c_2dot = (- A_2dot/norm_A + ( 2*np.dot(A*A_dot,A_dot) )/norm_A**3
        + np.dot( A_dot* A_dot + A*A_2dot ,A)/norm_A**3
        - 3*np.dot((A*A_dot)**2,A)/norm_A**5)

    b_ = np.cross(b3c, b1d)
    b_norm = la.norm(b_)
    b_dot = np.cross(b3c_dot, b1d) + np.cross(b3c, b1d_dot)
    b_2dot = (np.cross(b3c_2dot, b1d) + 2*np.cross(b3c_dot, b1d_dot)
        + np.cross(b3c, b1d_ddot))

    b1c = -np.cross( b3c, b_ )/b_norm
    b1c_dot = (-( np.cross(b3c_dot, b_)
            + np.cross(b3c, b_dot) )/b_norm
            + np.cross(b3c, b_)*(b_dot* b_)/b_norm**3)

    # intermediate steps to calculate b1c_2dot
    m_1 = ( np.cross(b3c_2dot, b_) + 2*np.cross(b3c_dot, b_dot)
            + np.cross(b3c, b_2dot) )/b_norm
    m_2 = ( np.cross(b3c_dot, b_)
            + np.cross(b3c, b_dot) )*np.dot(b_dot, b_)/b_norm**3
    m_dot = m_1 - m_2
    n_1 = np.cross(b3c, b_)*np.dot(b_dot, b_)
    n_1dot = (( np.cross(b3c_dot, b_) + np.cross(b3c, b_dot) )*np.dot(b_dot, b_)
        + np.cross(b3c, b_)*( np.dot(b_2dot, b_)+np.dot(b_dot, b_dot) ))
    n_dot = n_1dot/b_norm**3 - 3*n_1*np.dot(b_dot, b_)/b_norm**5
    b1c_2dot = -m_dot + n_dot

    Rc = np.reshape([b1c, np.cross(b3c, b1c), b3c],(3,3)).T
    Rc_dot = np.reshape([b1c_dot, ( np.cross(b3c_dot, b1c)
        + np.cross(b3c, b1c_dot) ), b3c_dot],(3,3)).T
    Rc_2dot = np.reshape( [b1c_2dot, ( np.cross(b3c_2dot, b1c)
        + np.cross(b3c_dot, b1c_dot) + np.cross(b3c_dot, b1c_dot)
        + np.cross(b3c, b1c_2dot) ), b3c_2dot],(3,3)).T
    Wc = att.vee_map(Rc.T.dot(Rc_dot))
    Wc_dot= att.vee_map( Rc_dot.T.dot(Rc_dot) + Rc.T.dot(Rc_2dot))
    return (Rc, Wc, Wc_dot) 
def attitude_errors( R, Rd, W, Wd ): 
    eR = 0.5* att.vee_map(Rd.T.dot(R) - R.T.dot(Rd)) 
    eW = W - R.T.dot(Rd.dot(Wd)) 
    return (eR, eW) 
def position_errors(x, xd, v, vd): 
    ex = x - xd 
    ev = v - vd 
    return (ex, ev) 
def rot_eul(x_in): 
    theta_x = np.arctan2(x_in[:,7], x_in[:,8]) 
    theta_y = np.arctan2(x_in[:,6], (x_in[:,7]**2+x_in[:,8]**2)**(1/2)) 
    theta_z = np.arctan2(x_in[:,1], x_in[:,0]) 
    return np.array([theta_x,theta_y,theta_z]).T 
def hat(x):
    hat_x = [0, -x[2], x[1], x[2], 0, -x[0], -x[1], x[0], 0] 
    return np.reshape(hat_x,(3,3)) 

