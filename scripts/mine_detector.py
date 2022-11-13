#!/usr/bin/env python
import rospy
import RPi.GPIO as GPIO
from std_srvs.srv import Empty, SetBool

rospy.init_node("mine_detector_node", anonymous = False)

GPIO.setmode(GPIO.BOARD)

metal_detector_pin = 38
prox_sensor_pin = 40

GPIO.setup(metal_detector_pin, GPIO.IN, pull_up_down = GPIO.PUD_UP)
GPIO.setup(prox_sensor_pin, GPIO.IN, pull_up_down = GPIO.PUD_UP)

grip_mine = rospy.ServiceProxy("grip_mine", Empty)
map_mine = rospy.ServiceProxy("map_mine", SetBool)

try:
    while not rospy.is_shutdown():
        if not GPIO.input(metal_detector_pin):
            if not GPIO.input(prox_sensor_pin):
                map_mine(True)
                try:
                    grip_mine()
                except rospy.service.ServiceException:
                    pass
                continue
            map_mine(False)

finally:
    GPIO.cleanup()

