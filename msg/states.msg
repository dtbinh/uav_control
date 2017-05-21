# header contains time stamp
Header header

# This message specifies the coordinate used for the program
# north east down (NED) or local tangent plane (LTP)
# east north up (ENU)
# desired position, velocity, acceleration in (ENU)
geometry_msgs/Vector3 xc
geometry_msgs/Vector3 xc_dot
geometry_msgs/Vector3 xc_2dot
geometry_msgs/Vector3 xc_3dot

# vicon pose, velocity (ENU)
geometry_msgs/Vector3 x_v
geometry_msgs/Quaternion q_v
geometry_msgs/Vector3 v_v

# IMU attitude, angular velocity (NED)
geometry_msgs/Vector3 w_imu
geometry_msgs/Quaternion q_imu

# desired attitude (NED)
float32[9] Rc
float32[9] Rc_dot
float32[9] Rc_2dot
geometry_msgs/Vector3 omega_c
geometry_msgs/Vector3 omega_c_dot
geometry_msgs/Vector3 b1d
geometry_msgs/Vector3 b1d_dot
geometry_msgs/Vector3 b1d_2dot

# position, attitude error vectors (NED)
geometry_msgs/Vector3 ex
geometry_msgs/Vector3 ev
geometry_msgs/Vector3 eR
geometry_msgs/Vector3 eW

# Actuation value
# force: total thrust
# f_motor: force for each motor
# f_motor_sat: saturation applied to the f_motor
# throttle: motor command value
# moment: in (NED)
float32 force
float32[4] f_motor
float32[4] f_motor_sat
float32[4] throttle
geometry_msgs/Vector3 Moment

# controller position and attitude gains
float32[4] gainX
float32[4] gainR

# sensor feed back from the motor
uint8[24] motor_power
