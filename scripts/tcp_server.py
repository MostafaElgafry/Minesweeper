#!/usr/bin/env python
import socket
import sys
import rospy
from minesweeper.srv import *

rospy.init_node('tcp_server_node', anonymous = False)
rospy.wait_for_service('pose_srv')
rospy.wait_for_service('cmd_vel')
get_pose = rospy.ServiceProxy('pose_srv', Pose_srv)
get_cmd_vel = rospy.ServiceProxy('cmd_vel', Cmd_srv)

# Create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# Listen for incoming connections
sock.bind(('', 6543))
sock.listen(1)

print >>sys.stderr, 'waiting for a connection'
connection, client_address = sock.accept()
print >>sys.stderr, 'connection from', client_address

try:
        # Receive the data in small chunks and retransmit it
    while True:
            data = connection.recv(32)
            if data:
                print >>sys.stderr, 'received "%s"' % data

            if data=='GET':
                resp = get_pose()
                msg = dict()
                msg['x'], msg['y'], msg['theta'] = \
                        resp.x, resp.y, resp.theta
                resp = get_cmd_vel()
                msg['lin'], msg['ang'] = resp.linear, resp.angular
                print >>sys.stderr, 'sending data back to the client'
                connection.sendall(str(msg))
            
finally:
        # Clean up the connection
        connection.close()
        sock.close()

