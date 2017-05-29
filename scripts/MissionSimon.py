#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon May 29 14:15:08 2017

@author: simonlibine
"""

#!/usr/bin/env python
import time
import rospy
from uav_control.msg import trajectory
import pygame
import sys
from trajectory_tracking_FOR_HADWARE import desired_pos, initialisation
pygame.init()
pygame.display.set_mode((300,300))

mission =  {'mode':'init'}
z_min = 0.15
z_hover = 1.5
v_up = 0.5

def get_key():
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            sys.exit()
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_w:
                mission['Forward'] = True
                print('Forward')
            elif event.key == pygame.K_s:
                mission['Simon'] = True
                print('Simon mission')
            elif event.key == pygame.K_q:
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

#while True:
#    get_key()
#    if states.get('quit') != None:
#        sys.exit()

pub = rospy.Publisher('xc', trajectory, queue_size= 10)
print('mode: t: takeoff, l: land, h: hover')
def mission_request():
    get_key()
    dt = 0.1
    t_init = time.time()
    cmd = trajectory()
    cmd.b1 = [1,0,0]
    cmd.header.frame_id = 'uav'
    if mission['mode'] == 'takeoff':
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
        t_total = 5
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
        mission['mode'] = 'wait'
        print('landing complete')
    elif mission['mode'] == 'hover':
        mission['mode'] = 'wait'
        pass
    elif mission['mode'] == 'Simon':
        print('Simon')
        t_total = 140
        t_cur = 0
        x0 = x_v #[0,0,0]
        dictionnary = initialisation(x0[0],x0[1],x0[2])
        while t_cur <= t_total:
            t_cur = time.time() - t_init
            time.sleep(dt)
            cmd.header.stamp = rospy.get_rostime()
            height = z_hover - v_up*t_cur
            desired_pos = desired_pos(t_cur,x_v,dictionnary)
            cmd.xc = desired_pos[0]
            cmd.xc_dot = desired_pos[1]
            cmd.xc_2dot = desired_pos[2]
            pub.publish(cmd)
            get_key()
        mission['mode'] = 'wait'
        print('Simon mission complete')
    else:
        pass
        #print('command not found: try again')

if __name__ == '__main__':
    try:
        rospy.init_node('command_station', anonymous=True)
        while True:
            mission_request()
    except rospy.ROSInterruptException:
        pass