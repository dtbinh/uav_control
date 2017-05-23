# header contains time stamp
Header header

# This message specifies the coordinate used for the program
# north east down (NED) or local tangent plane (LTP)
# east north up (ENU)
# desired position, velocity, acceleration in (ENU)
float32[3] xc # + xc_dot, xc_2dot, xc_3dot
float32[3] xc_dot
float32[3] xc_2dot
float32[3] xc_ned # desired command in NED frame
float32[3] xc_ned_dot
float32[3] xc_ned_2dot

# vicon pose, velocity (ENU)
float32[3] x_v
float32[4] q_v
float32[9] R_v
float32[3] v_v

# IMU attitude, angular velocity (NED)
float32[3] w_imu
float32[4] q_imu
float32[9] R_imu

# desired attitude (NED)
float32[9] Rc
float32[9] Rc_dot
float32[9] Rc_2dot
float32[6] omega_c # + omaga_c_dot
float32[3] b1d # + b1d_dot, b1d_2dot

# position attitude in NED
float32[3] x # x,v
float32[3] v
float32[9] R
float32[3] omega

# position, attitude error vectors (NED)
float32[3] ex # + ev
float32[3] ev
float32[3] eR # + eW
float32[3] eW
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
float32[3] moment

# controller position and attitude gains
float32[3] gain_position
float32[3] gain_attitude

# sensor feed back from the motor
uint8[12] motor_power
