import cython
import numpy as np
cimport numpy as np

cdef extern from 'src/controller.hpp':
    cdef cppclass controller:
        double kx, kv, kR, kW, m
        controller(double m, double* J, double* gains)
        void GeometricPositionController(double *x_in, double *v_in, double* R_in, double* W, double* xc, double* Rc_2dot, double* M_out)

cdef class c_control:
    cdef controller* c_control
    def __cinit__(self, double m, np.ndarray[double,ndim=1] J, np.ndarray[double,ndim=1] gains):
        self.c_control = new controller(m,<double*> J.data, <double*> gains.data)
    def position_control(self, np.ndarray[double, ndim=1] x_in, np.ndarray[double, ndim=1] v_in, np.ndarray[double, ndim=1] R_in, np.ndarray[double, ndim=1] W, np.ndarray[double, ndim=1] xc, np.ndarray[double, ndim=1] Rc_2dot, np.ndarray[double, ndim=1] M_out):
        self.c_control.GeometricPositionController(<double*> x_in.data, <double*> v_in.data,<double*> R_in.data, <double*> W.data, <double*> xc.data, <double*> Rc_2dot.data, <double*> M_out.data)
