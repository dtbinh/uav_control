import cython
cimport numpy as np
from libcpp cimport bool

cdef extern from "hw_interface.h":
    cdef cppclass hw_interface:
        hw_interface(int* address)
        int test()
        motor_command(int* thr, bool MotorWarmup, bool MOTOR_ON)
        int fhi2c
        open_I2C()

cdef class pyHW:
    cdef hw_interface* thisptr
    def __cinit__(self):
        self.thisptr = new hw_interface([0,0,0,0])
        pass
    def motor_command(self):
        pass
    def test(self):
        return self.thisptr.test()
    def open_I2C(self):
        pass

