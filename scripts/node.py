#!/usr/bin/env python
import rospy
import tf
import numpy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from controller import *

class uav(object):

    def __init__(self):
        rospy.init_node('uav')
        self.uav_pose = rospy.Subscriber('/vicon/Maya/pose',PoseStamped, self.mocap_sub)
        self.uav_w = rospy.Subscriber('/imu/imu',Imu, self.imu_sub)
        self.tf_subscriber = tf.TransformListener()
        self.x = None
        self.R = None
        self.W = None
        self.controller = Controller()
        rospy.spin()
    def mocap_sub(self, msg):
        try:
            (trans,rot) = self.tf_subscriber.lookupTransform('/world', '/Maya', rospy.Time(0))
            self.x = trans
            self.R = rot
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print('No transform between x and y found')
    def camera_sub(self):
        pass
    def imu_sub(self, msg):
        self.W = msg.angular_velocity
    def control(self):
        pass

if __name__ == '__main__':
    print('starting tf_subscriber node')
    uav_test = uav()
