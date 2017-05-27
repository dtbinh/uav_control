import cython
import numpy as np
cimport numpy as np

cdef extern from 'src/controller.hpp':
    cdef cppclass controller:
        void GeometricPositionController(double *xd)

cdef class c_control:
    cdef controller* c_control
    def __cinit__(self, double m, double g):
        self.c_control = new controller()
    def position_control(self, np.ndarray[double, ndim=1] a):
        self.c_control.GeometricPositionController(<double*> a.data)
