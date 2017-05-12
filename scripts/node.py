#!/usr/bin/env python
import rospy
import tf
import numpy as np
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from controller import *
import time
from i2c_cython.hw_interface import pyMotor


class uav(object):

    def __init__(self):
        rospy.init_node('uav')
        self.uav_pose = rospy.Subscriber('/vicon/Jetson/pose',PoseStamped, self.mocap_sub)
        self.uav_w = rospy.Subscriber('imu/imu',Imu, self.imu_sub)
        self.tf_subscriber = tf.TransformListener()
        self.tf = tf.TransformerROS(True,rospy.Duration(10.0))

        self.x = np.zeros(3)
        self.v = np.zeros(3)
        self.R = np.eye(3)
        self.W = np.zeros(3)
        b1 = np.array([1,0,0])
        self.x_c = (np.zeros(3), np.zeros(3),np.zeros(3),np.zeros(3),np.zeros(3),
                b1, np.zeros(3), np.zeros(3),
                np.eye(3),np.zeros(3),np.zeros(3))
        J = np.diag([0.0820, 0.0845, 0.1377])
        e3 = np.array([0.,0.,1.])
        self.controller = Controller(J,e3)
        self.F = None
        self.M = None

        self.hw_interface = pyMotor([44,42,42,43])
        rospy.spin()

    def mocap_sub(self, msg):
        try:
            (trans,rot) = self.tf_subscriber.lookupTransform('/world', '/Jetson', rospy.Time(0))
            self.x = trans
            self.R = self.tf.fromTranslationRotation(trans,rot)[:3,:3]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print('No transform between vicon and UAV found')

    def camera_sub(self):
        pass

    def imu_sub(self, msg):
        w = msg.angular_velocity
        self.W = np.array([w.x,w.y,w.z])
        self.control()

    def control(self):
        self.F, self.M = self.controller.position_control( self.R, self.W, self.x, self.v, self.x_c)

    def motor_command(self, command):
        self.hw_interface.motor_command(command, True)
        pass


if __name__ == '__main__':
    print('starting tf_subscriber node')
    uav_test = uav()
