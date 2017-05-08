#!/usr/bin/env python
import rospy
import tf
import numpy as np
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from controller import *

class uav(object):

    def __init__(self):
        rospy.init_node('uav')
        self.uav_pose = rospy.Subscriber('/vicon/Jetson/pose',PoseStamped, self.mocap_sub)
        self.uav_w = rospy.Subscriber('imu/imu',Imu, self.imu_sub)
        self.tf_subscriber = tf.TransformListener()
        self.x = np.zeros(3)
        self.v = np.zeros(3)
        self.R = np.eye(3)
        self.W = np.eye(3)
        e3 = np.array([0,0,1])
        self.x_c = (np.zeros(3), np.zeros(3),np.zeros(3),np.zeros(3),np.zeros(3),
                e3, np.zeros(3), np.zeros(3),
                np.eye(3),np.zeros(3),np.zeros(3))
        self.controller = Controller(np.eye(3),np.ones(3))
        rospy.spin()
    def mocap_sub(self, msg):
        try:
            (trans,rot) = self.tf_subscriber.lookupTransform('/world', '/Jetson', rospy.Time(0))
            self.x = trans
            # self.R = rot
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print('No transform between x and y found')
    def camera_sub(self):
        pass
    def imu_sub(self, msg):
        w = msg.angular_velocity
        self.W = np.array([w.x,w.y,w.z])
        self.control()
    def control(self):
        # print(self.controller.position_control( self.R, self.W, self.x, self.v, self.x_c))
        pass

if __name__ == '__main__':
    print('starting tf_subscriber node')
    uav_test = uav()
