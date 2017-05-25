from hw_interface import pyMotor
import sys
import time
import numpy as np
test_motor = pyMotor([44,41,42,43])
print(test_motor.get_motor_address())
test_motor.pyTest([1,2,3])
# test_motor.open_i2c()
# sys.exit()

def prettyPrint(data):
    print(['{:.1f}'.format(i) for i in np.array(data).flatten()])

for i in range(4):
    throttle = [0,0,0,0]
    throttle[i] = 50
    for k in range(100):
        prettyPrint(test_motor.motor_command(throttle,True))
        time.sleep(0.01)
for i in range(400):
    prettyPrint(test_motor.motor_command([30,30,30,30],True))
    time.sleep(0.01)
# test_motor.motor_command([20,20,20,20], False,False)
# print(MoTest.pyTest([3,1,2]))

