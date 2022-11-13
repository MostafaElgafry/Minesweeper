#!/usr/bin/env python
import RPi.GPIO as gpio
from time import sleep
import rospy 
import time 
from sensor_msgs.msg import Joy

pwm_pin=24
A0=32
A1=26
magnetic=36
gpio.setmode(gpio.BOARD)
gpio.setup(pwm_pin,gpio.OUT)
pwm=gpio.PWM(pwm_pin,10000)
pwm.start(0.0)
gpio.setup(A0,gpio.OUT)
gpio.setup(A1,gpio.OUT)
gpio.setup(magnetic,gpio.OUT)
def callback (msg):
    if msg.axes[4]>0.00:
        #workcycle=msg.axes[4]*100.00
        pwm.ChangeDutyCycle(45)
        gpio.output(A0,True)
        gpio.output(A1,False)
        print("\nup")
    if msg.axes[4]<0.00:
        #workcycle=msg.axes[4]*-100.00
        pwm.ChangeDutyCycle(45)
        gpio.output(A0,False)
        gpio.output(A1,True)
        print("\down")
    if msg.buttons[5]==True:
        gpio.output(magnetic,True)
        print("\nmagnetic_on")
    if msg.buttons [4]==False:
        gpio.output(magnetic,False)
        print("\nmagnetic_off")
    if msg.axes[4]==0:
        pwm.ChangeDutyCycle(0.0)
def listener():
    rospy.init_node("manual_arm",anonymous=False)
    rospy.Subscriber("/joy",Joy,callback)
    rospy.spin()
if __name__=="__main__":
    try:
        listener()
    finally:
        gpio.cleanup()
