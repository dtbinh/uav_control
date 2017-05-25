#!/usr/bin/env python

import rospy
from dynamic_reconfigure.server import Server
from uav_control.cfg import gainsConfig

def callback(config, level):
    rospy.loginfo('config update')
    return config

if __name__=='__main__':
    rospy.init_node('gain_tuning',  anonymous = True)
    srv = Server(gainsConfig, callback)
    rospy.spin()
