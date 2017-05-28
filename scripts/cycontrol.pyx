import cython
import numpy as np
cimport numpy as np

cdef extern from 'src/controller.hpp':
    cdef cppclass controller:
        double kx, kv, kR, kW, m
        controller(double m,double dt, double* J, double* gains)
        void GeometricPositionController(double *x_in, double *v_in, double* R_in, double* W, double* xc, double* M_out)
        void get_Rc(double* R)
        void get_Rc_dot(double* R)
        void get_Rc_2dot(double* R)
        void get_e3(double* R)
        void get_ex(double* R)
        void get_ev(double* R)
        void get_eR(double* R)
        void get_eW(double* R)
        void get_Wc(double* R)
        void get_Wc_dot(double* R)
        void get_eiR(double* R)
        void get_b1d(double* R)

cdef class c_control:

    cdef controller* c_control

    def __cinit__(self, double m, double dt,np.ndarray[double,ndim=1] J, np.ndarray[double,ndim=1] gains):
        self.c_control = new controller( m, dt, <double*> J.data, <double*> gains.data)

    def position_control(self, np.ndarray[double, ndim=1] x_in, np.ndarray[double, ndim=1] v_in, np.ndarray[double, ndim=1] R_in, np.ndarray[double, ndim=1] W, np.ndarray[double, ndim=1] xc, np.ndarray[double, ndim=1] M_out):
        self.c_control.GeometricPositionController(<double*> x_in.data, <double*> v_in.data,<double*> R_in.data, <double*> W.data, <double*> xc.data, <double*> M_out.data)

    @property
    def kx(self):
        return self.c_control.kx
    @kx.setter
    def kx(self, val):
        self.c_control.kx = val

    @property
    def kv(self):
        return self.c_control.kv
    @kv.setter
    def kv(self, val):
        self.c_control.kv = val

    @property
    def kR(self):
        return self.c_control.kR
    @kR.setter
    def kR(self, val):
        self.c_control.kR = val

    @property
    def kW(self):
        return self.c_control.kW
    @kW.setter
    def kW(self, val):
        self.c_control.kW = val

    def get_b1d(self, np.ndarray[double,ndim =1] R):
        self.c_control.get_b1d(<double*> R.data)

    def get_Rc(self, np.ndarray[double,ndim =1] R):
        self.c_control.get_Rc(<double*> R.data)

    def get_Rc_dot(self, np.ndarray[double,ndim =1] R):
        self.c_control.get_Rc_dot(<double*> R.data)

    def get_Rc_2dot(self, np.ndarray[double,ndim =1] R):
        self.c_control.get_Rc_2dot(<double*> R.data)

    def get_e3(self, np.ndarray[double,ndim =1] R):
        self.c_control.get_e3(<double*> R.data)

    def get_ex(self, np.ndarray[double,ndim =1] R):
        self.c_control.get_ex(<double*> R.data)

    def get_ev(self, np.ndarray[double,ndim =1] R):
        self.c_control.get_ev(<double*> R.data)

    def get_eR(self, np.ndarray[double,ndim =1] R):
        self.c_control.get_eR(<double*> R.data)

    def get_eW(self, np.ndarray[double,ndim =1] R):
        self.c_control.get_eW(<double*> R.data)

    def get_Wc(self, np.ndarray[double,ndim =1] R):
        self.c_control.get_Wc(<double*> R.data)

    def get_Wc_dot(self, np.ndarray[double,ndim =1] R):
        self.c_control.get_Wc_dot(<double*> R.data)

    def get_eiR(self, np.ndarray[double,ndim =1] R):
        self.c_control.get_eiR(<double*> R.data)
