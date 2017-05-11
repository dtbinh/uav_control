from MoTest import pyMotor

test_motor = pyMotor([44,41,42,43])
print(test_motor.get_motor_address())
test_motor.pyTest([1,2,3])
# test_motor.open_i2c()
for i in range(4):
    test_motor.motor_test(i,30)
# test_motor.motor_command([20,20,20,20], False,False)
# print(MoTest.pyTest([3,1,2]))

