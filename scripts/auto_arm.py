#!/usr/bin/env python
import RPi.GPIO as gpio
from time import sleep
from minesweeper.msg import Mines_msg
from sensor_msgs.msg import Joy
import rospy 
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
import sys

def grip_mine_handler(req):
    global dir1, dir2, pwm, limit_switch1, limit_switch2, magnetic
    sleep(2)
    print("\nmagnetic_on")
    gpio.output(magnetic,True)
    while gpio.input(limit_switch1): ##up
        print("\up")
        pwm.ChangeDutyCycle(50)
        gpio.output(dir1,True)
        gpio.output(dir2,False)
    pwm.ChangeDutyCycle(0.0)
    gpio.output(magnetic,False)
    print("\nmagnetic_off")
    sleep(1)                     
    while gpio.input(limit_switch2):                  ##dowm
        pwm.ChangeDutyCycle(50)
        gpio.output(dir1,False)
        gpio.output(dir2,True)
    pwm.ChangeDutyCycle(0.0)
    return EmptyResponse()

pwm_pin=24
dir1=32
dir2=26
magnetic=36
limit_switch1=13
limit_switch2=15

gpio.setmode(gpio.BOARD)
gpio.setwarnings(False)
gpio.setup(pwm_pin,gpio.OUT)
pwm=gpio.PWM(pwm_pin,10000)
pwm.start(0.0)

gpio.setup(dir1,gpio.OUT)
gpio.setup(dir2,gpio.OUT)
gpio.setup(magnetic,gpio.OUT)
gpio.setup(limit_switch1,gpio.IN,pull_up_down=gpio.PUD_UP)
gpio.setup(limit_switch2,gpio.IN,pull_up_down=gpio.PUD_UP)

def callback2(msg):
    if msg.buttons[2]:
        print("\nauto_is_off")
        global ser
        ser.shutdown()

if __name__=="__main__":
    try:
        rospy.init_node("auto_arm_node",anonymous=False)
        ser = rospy.Service("grip_mine", Empty, grip_mine_handler)
        rospy.Subscriber("joy",Joy,callback2)
        rospy.spin()

    except KeyboardInterrupt:
        gpio.cleanup()
