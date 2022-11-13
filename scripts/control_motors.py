#!/usr/bin/env python
import RPi.GPIO as gpio
from time import sleep
import rospy
import time
from sensor_msgs.msg import Joy
from minesweeper.srv import *
########################
## +5v=2   -> +5 vcc  ##
## pwm1=10  -> 5       ##
## pwm2=12  -> 6       ##
## A0=18    -> 7       ##
## GND=9   -> GND     ##
## A1=16   -> 8       ##
## B0=22   -> 4       ## 
## B1=8   -> 9       ##
########################

pwm1=10
pwm2=12
A0=18
A1=16
B0=22
B1=8
gpio.setmode(gpio.BOARD)

gpio.setup(pwm1,gpio.OUT)      #pwm1_pin
gpio.setup(pwm2,gpio.OUT)      #pwm2_pin
pwm1=gpio.PWM(pwm1,10000)
pwm2=gpio.PWM(pwm2,10000)
pwm1.start(0.0)
pwm2.start(0.0)

gpio.setup(A0,gpio.OUT)        ### Motor1 ###
gpio.setup(A1,gpio.OUT)        ### Motor1 ###

gpio.setup(B0,gpio.OUT)        ### Motor2 ###
gpio.setup(B1,gpio.OUT)        ### Motor2 ###
gpio.setwarnings(False)
###########################################################

cmd_vel = {'lin':0.0, 'ang':0.0}

def callback(msg):
    global cmd_vel
    if msg.axes[1]>0.00:                     ###forward###
        workcycle=msg.axes[1]*100.00
        cmd_vel = {'lin': workcycle/100.0, 'ang':0.0}
        pwm1.ChangeDutyCycle(workcycle)
        pwm2.ChangeDutyCycle(workcycle)
        gpio.output(A0,True)
        gpio.output(A1,False)
        gpio.output(B0,True)
        gpio.output(B1,False)        
        print("forward\n")

    elif msg.axes[1]<0.00:                   ###backward###
        workcycle=msg.axes[1]*-100.00
        cmd_vel = {'lin': workcycle/100.0, 'ang':0.0}
        pwm1.ChangeDutyCycle(workcycle)
        pwm1.ChangeDutyCycle(workcycle)
        pwm2.ChangeDutyCycle(workcycle)
        gpio.output(A0,False)
        gpio.output(A1,True)
        gpio.output(B0,False)
        gpio.output(B1,True)
        print("backward\n")

    elif msg.axes[0]>0.00:                   ###left###
        workcycle=msg.axes[0]*100.00
        cmd_vel = {'lin': 0.0, 'ang': workcycle/100.0}
        pwm1.ChangeDutyCycle(workcycle)
        pwm2.ChangeDutyCycle(workcycle)
        gpio.output(A0,True)
        gpio.output(A1,False)
        gpio.output(B0,False)
        gpio.output(B1,True)
        print("left\n")

    elif msg.axes[0]<0.00:                   ###right###
        workcycle=msg.axes[0]*-100.00
        cmd_vel = {'lin': 0.0, 'ang':workcycle/100.0}
        pwm1.ChangeDutyCycle(workcycle)
        pwm2.ChangeDutyCycle(workcycle)
        gpio.output(A0,False)
        gpio.output(A1,True)
        gpio.output(B0,True)
        gpio.output(B1,False)
        print("right\n")
    else:
        cmd_vel = {'lin': 0.0, 'ang': 0.0}
        pwm1.ChangeDutyCycle(0)
        pwm2.ChangeDutyCycle(0)

def get_cmd(req):
    return Cmd_srvResponse(cmd_vel['lin'], cmd_vel['ang'])

def listener():    
    rospy.init_node("control_motors",anonymous=True)
    rospy.Subscriber("/joy",Joy,callback)
    rospy.Service("cmd_vel", Cmd_srv, get_cmd)
    rospy.spin()


#############################################################

if __name__== "__main__":
    try:
        listener()
    finally:
        gpio.cleanup()


