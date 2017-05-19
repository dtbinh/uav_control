#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import PoseStamped
import numpy as np


class degrade(object):
    def __init__(self):
        rospy.init_node('degrade_mocap')
        # self.pub = rospy.Publisher('/Jetson')
        self.tf_subscriber = tf.TransformListener()
        rospy.Subscriber('/vicon/Jetson/pose', PoseStamped, self.mocap_sub)
        self.pub = rospy.Publisher('degrated_tf', PoseStamped, latch=True)
        self.tf = tf.TransformerROS(True,rospy.Duration(10.0))
        self.x = np.zeros(3)
        self.orientation = np.zeros(4)
        self.rostime = None


    def mocap_sub(self, msg):
        ori = msg.pose.orientation
        self.orientation = np.array([ori.x,ori.y,ori.z,ori.w])
        self.rostime = msg.header.stamp
        try:
            (trans,rot) = self.tf_subscriber.lookupTransform('/world', '/Jetson', rospy.Time())
            self.x = trans
            self.R = self.tf.fromTranslationRotation(trans,rot)[:3,:3]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print('No transform between vicon and UAV found')

    def publish(self):
        rate = rospy.Rate(3)
        br = tf.TransformBroadcaster()
        while not rospy.is_shutdown():
            try:
                br.sendTransform( self.x, self.orientation,
                self.rostime,
                'degraded_Jetson',
                'world')
                rate.sleep()
            except (rospy.exceptions.ROSTimeMovedBackwardsException):
                pass

if __name__ == '__main__':
    testobj = degrade()
    testobj.publish()
