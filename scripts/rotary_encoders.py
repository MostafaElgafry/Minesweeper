#!/usr/bin/env python
import RPi.GPIO as GPIO
import rospy
from minesweeper.msg import Pose_msg
from minesweeper.srv import *
from math import cos, sin, pi
from time import sleep

dis_res = .05
width = 13.5
ang_res = dis_res / width

x = 0.0
y = 0.0
theta = 0.0

class Rotary (object):

    def __init__(self, RoA, RoB):
        
        self.RoA = RoA
        self.RoB = RoB
        
        GPIO.setup(RoA, GPIO.IN)
        GPIO.setup(RoB, GPIO.IN)

        GPIO.add_event_detect(RoA, GPIO.RISING,\
                self.interrupt_handler)

    def interrupt_handler(self, _):

        if GPIO.input(self.RoB) == GPIO.HIGH:
            dis_direc = 1
        else:
            dis_direc = -1
        
        global counter1, counter2

        if self.RoA == 21:
            ang_direc = 1

        else:
            ang_direc = -1

        global x, y, theta, dis_res, ang_res
        
        theta_av = theta + .5 * ang_direc * ang_res
        theta = theta + ang_direc * ang_res

        x = x + dis_res * dis_direc * cos(theta_av) * .5
        y = y + dis_res * dis_direc * sin(theta_av) * .5
        
        if theta > pi:
            theta = theta - 2 * pi
        elif theta < -pi:
            theta = theta + 2 * pi

        msg = Pose_msg()
        msg.x = x
        msg.y = y
        
        global pub
        pub.publish(msg)
        print x, y, theta

def pose_srv_handler(req):
    global x, y, theta
    return Pose_srvResponse(x, y, theta)


def reset_pose(msg):
    global x, y, theta
    x = msg.x
    y = msg.y
    theta = msg.theta

rospy.init_node("pose_node", anonymous=False)
pub = rospy.Publisher ("/minesweeper/pose", Pose_msg, queue_size=10)
ser = rospy.Service('pose_srv', Pose_srv, pose_srv_handler)
rospy.Subscriber('/minesweeper/camera_pose', Pose_msg, reset_pose)

def destroy():

	GPIO.cleanup()             # Release resource

if __name__ == '__main__':     # Program start from here
    
    GPIO.setmode (GPIO.BOARD)
    
    r1 = Rotary(3, 5)
    r2 = Rotary(7, 11)

    try:
        rospy.spin()

    except KeyboardInterrupt:
        destroy()

