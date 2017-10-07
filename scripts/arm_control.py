#!/usr/bin/env python

import sys
import rospy
import geometry_msgs
from uarm.srv import *


def moveToclient(x, y, z):
    rospy.wait_for_service('uarm/move_to')
    try:
        move_to = rospy.ServiceProxy('uarm/move_to', MoveTo)
        # Get position from geometry_msgs/Point
        pos = geometry_msgs.msg.Point()
        pos.x=float(x)
        pos.y=float(y)
        pos.z=float(z)
        
        # Taken from the rqt_gui
        eef_orientation = float(0.0)
        move_mode = int(0)
        ignore_orientation = bool(False)
        interpolation_type = int(0)
        check_limits = bool(False)
        
        # Builtin duration type from ROS
        # Duration(secs, nsecs) 
        d = rospy.Duration(0,100)
        movement_duration = d
		
		# Use the function move_to
        resp1 = move_to(pos, eef_orientation, move_mode, movement_duration, ignore_orientation, interpolation_type, check_limits) 
    
        return resp1
    except rospy.ServiceException, e:
        print "MoveTo service call failed: %s"%e

if __name__ == "__main__":
	# Change the x, y, z coordinates for the arm
	# This is where we can subscribe to the coordinates from
	# the camera 
    x=10
    y=0
    z=0
    print moveToclient(x,y,z)
    
