from MoTest import pyMotor

test_motor = pyMotor([40,41,42,43])
print(test_motor.get_motor_address())
test_motor.pyTest([1,2,3])
test_motor.open_i2c()
test_motor.motor_command([20,20,20,20], False,False)
# print(MoTest.pyTest([3,1,2]))

