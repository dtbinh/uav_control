import cython
import numpy as np
cimport numpy as np

cdef extern from 'src/controller.hpp':
    cdef cppclass controller:
        double kx, kv, kR, kW, m
        controller(double m, double* J, double* gains)
        void GeometricPositionController(double *x_in, double *v_in, double* R_in, double* W, double* xc, double* M_out)
        void get_Rc(double* R)
        void get_Rc_dot(double* R)
        void get_Rc_2dot(double* R)

cdef class c_control:
    cdef controller* c_control
    def __cinit__(self, double m, np.ndarray[double,ndim=1] J, np.ndarray[double,ndim=1] gains):
        self.c_control = new controller(m,<double*> J.data, <double*> gains.data)
    def position_control(self, np.ndarray[double, ndim=1] x_in, np.ndarray[double, ndim=1] v_in, np.ndarray[double, ndim=1] R_in, np.ndarray[double, ndim=1] W, np.ndarray[double, ndim=1] xc, np.ndarray[double, ndim=1] M_out):
        self.c_control.GeometricPositionController(<double*> x_in.data, <double*> v_in.data,<double*> R_in.data, <double*> W.data, <double*> xc.data, <double*> M_out.data)
    def get_kx(self):
        return self.c_control.kx
    def get_Rc(self, np.ndarray[double,ndim =1] R):
        self.c_control.get_Rc(<double*> R.data)
    def get_Rc_dot(self, np.ndarray[double,ndim =1] R):
        self.c_control.get_Rc_dot(<double*> R.data)
    def get_Rc_2dot(self, np.ndarray[double,ndim =1] R):
        self.c_control.get_Rc_2dot(<double*> R.data)
