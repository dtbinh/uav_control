#!/usr/bin/env python
import time
import rospy
from uav_control.msg import trajectory
import pygame
import sys
from trajectory_tracking_FOR_HADWARE import desired_pos, initialisation
from geometry_msgs.msg import PoseStamped
import numpy as np
pygame.init()
pygame.display.set_mode((300,300))

mission =  {'mode':'init','motor':False,'warmup':False}
z_min = 0.35
z_hover = 1.5
v_up = 0.5
x_v = [0,0,0]
x_ship = [0,0,0]

def mocap_sub(msg):
    global x_v
    x = msg.pose.position
    x_v = np.array([x.x,x.y,x.z])

def mocap_sub_ship(msg):
    global x_ship
    x = msg.pose.position
    x_ship = np.array([x.x,x.y,x.z])

def get_key():
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            rospy.set_param('/odroid_node/Motor', False)
            sys.exit()
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_f:
                mission['Forward'] = True
                print('Forward')
            elif event.key == pygame.K_r:
                mission['mode'] = 'reset'
                print('Resetting the rosparams')
                rospy.set_param('/odroid_node/MotorWarmup', False)
                rospy.set_param('/odroid_node/Motor', False)
            elif event.key == pygame.K_s:
                mission['mode'] = 'Simon'
                print('Simon mission')
            elif event.key == pygame.K_q:
                rospy.set_param('/odroid_node/Motor', False)
                sys.exit()
            elif event.key == pygame.K_t:
                mission['mode'] = 'takeoff'
                print('takeoff')
            elif event.key == pygame.K_l:
                mission['mode'] = 'land'
                print('Landing')
            elif event.key == pygame.K_h:
                mission['mode'] = 'hover'
                print('Hovering at origin')
            elif event.key == pygame.K_m:
                mission['motor'] = not mission['motor']
                print('Motor on')
            elif event.key == pygame.K_w:
                mission['warmup'] = not mission['warmup']
                print('Motor warmup')


pub = rospy.Publisher('xc', trajectory, queue_size= 10)
print('mode: t: takeoff, l: land, h: hover, s: Simon, m: motor, w: warmup, r: reset')
def mission_request():
    global x_v
    get_key()
    dt = 0.1
    t_init = time.time()
    cmd = trajectory()
    cmd.b1 = [1,0,0]
    cmd.header.frame_id = 'uav'

    if mission['motor'] == True:
        if rospy.get_param('/odroid_node/Motor'):
            rospy.set_param('/odroid_node/Motor', False)
            print('Motor OFF')
        else:
            rospy.set_param('/odroid_node/Motor', True)
            print('Motor ON')
        rospy.set_param('/odroid_node/MotorWarmup', True)
        pub.publish(cmd)
        mission['motor'] = False

    elif mission['warmup'] == True:
        if rospy.get_param('/odroid_node/MotorWarmup'):
            rospy.set_param('/odroid_node/MotorWarmup', False)
            print('Motor warmup OFF')
        else:
            rospy.set_param('/odroid_node/MotorWarmup', True)
            print('Motor warmup ON')
        pub.publish(cmd)
        mission['warmup'] = False

    if mission['mode'] == 'takeoff':
        rospy.set_param('/odroid_node/Motor', True)
        rospy.set_param('/odroid_node/MotorWarmup', True)
        print('Motor warmup ON')
        rospy.sleep(4)
        rospy.set_param('/odroid_node/MotorWarmup', False)
        print('Taking off at {} sec'.format(time.time()-t_init))
        t_total = 5
        t_cur= 0
        while t_cur <= t_total:
            t_cur = time.time() - t_init
            time.sleep(dt)
            cmd.header.stamp = rospy.get_rostime()
            height = z_min+v_up*t_cur
            cmd.xc = [0,0,height if height < 1.5 else 1.5 ]
            cmd.xc_dot = [0,0,v_up]
            pub.publish(cmd)
            get_key()
        mission['mode'] = 'wait'
        print('Take off complete')

    elif mission['mode'] == 'land':
        print('Landing')
        t_total = 3
        t_cur = 0
        while t_cur <= t_total:
            t_cur = time.time() - t_init
            time.sleep(dt)
            cmd.header.stamp = rospy.get_rostime()
            height = z_hover - v_up*t_cur
            cmd.xc = [0,0,height if height > z_min else 0]
            cmd.xc_dot = [0,0,-v_up]
            pub.publish(cmd)
            get_key()
            if x_v[2] < z_min:
                rospy.set_param('/odroid_node/Motor', False)
        rospy.set_param('/odroid_node/Motor', False)
        mission['mode'] = 'wait'
        print('landing complete')

    elif mission['mode'] == 'Simon':
        rospy.set_param('/odroid_node/Motor', True)
        rospy.set_param('/odroid_node/MotorWarmup', True)
        print('Motor warmup ON')
        rospy.sleep(4)
        rospy.set_param('/odroid_node/MotorWarmup', False)
        print('Simon')
        t_total = 140
        t_cur = 0
        t_init = time.time()
        x0 = x_v #[0,0,0]
        dictionnary = initialisation(x0[0],x0[1],x0[2])
        while True:
            t_cur = time.time() - t_init
            time.sleep(dt)
            cmd.header.stamp = rospy.get_rostime()
            height = z_hover - v_up*t_cur
            d_pos = desired_pos(t_cur,x_v,dictionnary, x_ship)
            cmd.xc = d_pos[0]
            cmd.xc_dot = d_pos[1]
            cmd.xc_2dot = d_pos[2]
            pub.publish(cmd)
            get_key()
	    if x_v[2] < z_min and t_cur > 5:
        	rospy.set_param('/odroid_node/Motor', False)
        mission['mode'] = 'wait'
        print('Simon mission complete')

    elif mission['mode'] == 'hover':
        mission['mode'] = 'wait'
        pass

    else:
        pass
        #print('command not found: try again')

if __name__ == '__main__':
    try:
        rospy.init_node('command_station', anonymous=True)
        uav_pose = rospy.Subscriber('/vicon/Maya/pose',PoseStamped, mocap_sub)
        ship_pose = rospy.Subscriber('/vicon/ship/pose',PoseStamped, mocap_sub_ship)
        while True:
            mission_request()
    except rospy.ROSInterruptException:
        pass
