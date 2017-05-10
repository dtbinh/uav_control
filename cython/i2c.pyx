import cython
from libcpp cimport bool
from libcpp.vector cimport vector

cdef extern from "hw_interface.h":
    cdef cppclass hw_interface:
        hw_interface()
        vector[int] mtr_addr
        int test()
        motor_command(int* thr, bool MotorWarmup, bool MOTOR_ON)
        int fhi2c
        vector[int] get_mtrAddr()
        open_I2C()

cdef class pyHW:
    cpdef hw_interface* thisptr
    def __cinit__(self):
        self.thisptr = new hw_interface()
        pass
    def motor_command(self):
        pass
    def test(self):
        return self.thisptr.test()
    def get_mtrAddr(self):
        cdef vector[int] address
        address = self.thisptr.get_mtrAddr()
        b = []
        for i in range(4):
            b.append(address[i])
        return b
    def open_I2C(self):
        pass
    def pytest(self):
        return 5

