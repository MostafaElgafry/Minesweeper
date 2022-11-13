#!/usr/bin/env python
import rospy
from minesweeper.srv import *
from sensor_msgs.msg import Joy

rospy.init_node("map_generator_node", anonymous = False)
rospy.Subscriber("joy", Joy)

def map_mine(msg):
    if not (msg.buttons[] or msg.buttons[]):
        return
    global get_pose
    resp = get_pose()
    x, y = int(resp.x/100.0), int(resp.y/100.0)
    line = str(x) + " " + str(y) + " "

    if msg.buttons[]:
        state = "surface"
    else:
        state = "burried"
    line += state
    line += "\n"
    map_file = open("/home/minesweeper/map.txt", "a")
    map_file.write(line)
    map_file.close()

get_pose = rospy.ServiceProxy("pose_srv", Pose_srv)

rospy.spin()
