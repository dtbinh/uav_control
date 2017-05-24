#!/usr/bin/env python
import time
import rospy
from geometry_msgs.msg import Quaternion
from uav_control.msg import trajectory
#import pygame
#pygame.init()
#pygame.display.set_caption('mission')
#size= [640,480]
#screen = pygame.display.set_mode(size)
#clock = pygame.time.Clock()

pub = rospy.Publisher('xd', trajectory, queue_size= 10)
def mission():
    dt = 0.01
    mode = int(input('Flight mode: '))
    t_init = time.time()
    cmd = trajectory()
    cmd.b1 = [1,0,0]
    cmd.header.frame_id = 'uav'
    if mode == 0:
        print('Taking off at {} sec'.format(time.time()-t_init))
        t_total = 5
        t_cur= 0
        try:
            while t_cur <= t_total:
                t_cur = time.time() - t_init
                time.sleep(dt)
                cmd.header.stamp = rospy.get_rostime()
                pub.publish(cmd)
                print(t_cur)
                if input() == 1:
                    break
        except KeyboardInterrupt:
            print('mission terminated')
    elif mode == 1:
        print('Landing')
        t_total = 5
        t_cur = 0
        while t_cur <= t_total:
            t_cur = time.time() - t_init
            time.sleep(dt)
    elif mode == 2:
        while True:
            pressed = pygame.key.get_pressed()
            if pressed[pygame.K_w]:
                print('w pressed')
                break
        pass
    else:
        print('command not found: try again')

if __name__ == '__main__':
    try:
        rospy.init_node('command_station', anonymous=True)
        while True:
            mission()
    except rospy.ROSInterruptException:
        pass
