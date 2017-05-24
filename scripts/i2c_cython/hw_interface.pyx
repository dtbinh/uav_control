from libcpp.vector cimport vector
from libcpp cimport bool

cdef extern from "hw_interface.c":
    cdef cppclass hw_interface:
        hw_interface(vector[int])
        void single_motor_test(int,int)
        # void rotateMotors(int, vector[int], bool)
        vector[int] motor_command(vector[int], bool)
        void open_i2c()
        vector[int] get_motor_address()
        vector[int] test(vector[int])

cdef class pyMotor:
    cdef hw_interface* thisptr
    def __cinit__(self, address):
        self.thisptr = new hw_interface(address)
    def __dealloc__(self):
        del self.thisptr
    def rotateMotors(self, motor, throttle, motor_on):
        # self.thisptr.rotateMotors(motor, throttle, motor_on)
        pass
    def motor_test(self, int motor_id, throttle):
        self.thisptr.single_motor_test(motor_id, throttle)
    def motor_command(self, throttle, motor_on):
        cdef sensor_val = self.thisptr.motor_command(throttle, motor_on)
        cdef sensor_out = []
        for i in range(4):
            sensor_out.append([sensor_val[i*6],sensor_val[i*6+3],sensor_val[i*6+5]])
        return sensor_out
    def open_i2c(self):
        self.thisptr.open_i2c()
    def get_motor_address(self):
        return self.thisptr.get_motor_address()
    def pyTest(self,msg):
        return self.thisptr.test(msg)
