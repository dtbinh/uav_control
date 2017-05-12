from hw_interface import pyMotor
import sys

test_motor = pyMotor([44,41,42,43])
print(test_motor.get_motor_address())
test_motor.pyTest([1,2,3])
# test_motor.open_i2c()
# sys.exit()
for i in range(400):
    print(test_motor.motor_command([30,30,30,30],True))
# test_motor.motor_command([20,20,20,20], False,False)
# print(MoTest.pyTest([3,1,2]))

