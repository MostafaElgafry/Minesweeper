#!/usr/bin/env python
import rospy
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from minesweeper.srv import *

rospy.init_node("map_generator_node", anonymous = False)
map_dict = dict()
for x in range(0, 10):
    for y in range(0,10):
        map_dict([x, y]) = "clear"

def map_mine(req):
    global get_pose
    resp = get_pose()
    x, y = int(resp.x/100.0), int(resp.y/100.0)
    last_state = map_dict([x, y])
    if last_state=="surface":
        return SetBoolResponse()
    state = 'surface' * req.data + 'burried' * ~req.data
    
    if state == last_state:
        return SetBoolResponse()
    
    map_dict([x, y]) = state

    line = str(x) + " " + str(y) + " " + state
    line += "\n"
    map_file = open("/home/minesweeper/map.txt", "a")
    map_file.write(line)
    map_file.close()
    
    resp = SetBoolResponse()
    resp.success = True
    return resp
    
ser = rospy.Service("map_mine", SetBool, map_mine)
get_pose = rospy.ServiceProxy("pose_srv", Pose_srv)

rospy.spin()
