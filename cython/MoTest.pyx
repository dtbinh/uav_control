from libcpp.vector cimport vector
from libcpp cimport bool

cdef extern from "MotorTest.h":
    cdef cppclass hw_interface:
        hw_interface(vector[int])
        vector[int] motor_command(vector[int],bool,bool)
        void open_i2c()
        vector[int] get_motor_address()
        vector[int] test(vector[int])

cdef class pyMotor:
    cdef hw_interface* thisptr
    def __cinit__(self, address):
        self.thisptr = new hw_interface(address)
    def __dealloc__(self):
        del self.thisptr
    def motor_command(self, throttle, motor_warmup, motor_on):
        return self.thisptr.motor_command(throttle, motor_warmup, motor_on)
    def open_i2c(self):
        self.thisptr.open_i2c()
    def get_motor_address(self):
        return self.thisptr.get_motor_address()
    def pyTest(self,msg):
        return self.thisptr.test(msg)
