#!/usr/bin/env python

import sys
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import String
from uarm.srv import *
import math
import ros_kth_uarm.kth_uarm as kth_uarm

# get limits and calibration config from separate file
LOWER_LIMITS= kth_uarm.KTHUarm.LOWER_LIMITS
UPPER_LIMITS= kth_uarm.KTHUarm.UPPER_LIMITS
RESET_POS= kth_uarm.KTHUarm.CALIBRATION_CONFIG
J_1_ZERO=-30.0 # zero angle for j1
J_2_ZERO=-7 # zero angle for j2
# LINKS
L1 = 10.645
L2 = 2.117
L3 = 14.825
L4 = 16.02
L5 = 3.3
L6 = 5.9

# reset arm to neutral position
def resetPosition():
    print "Arm position reset..."
    resp= moveToJointsClient(RESET_POS[0], RESET_POS[1], RESET_POS[2], int(1), 1)
    if resp.error:
        print "Error, could not reset arm position. Check limits."

# move to service
def moveToClient(x, y, z, interpolate, seconds):
    rospy.wait_for_service('uarm/move_to')
    try:
        move_to = rospy.ServiceProxy('uarm/move_to', MoveTo)
        # Get position from geometry_msgs/Point
        pos = Point()
        pos.x=float(x)
        pos.y=float(y)
        pos.z=float(z)
        
        # Taken from the rqt_gui
        eef_orientation = float(0.0)
        move_mode = int(0)
        ignore_orientation = bool(False)
        interpolation_type = interpolate
        check_limits = bool(True)
        
        # Built-in duration type from ROS
        # Duration(secs, nsecs) 
        d = rospy.Duration(seconds,0)
        movement_duration = d
		
		# Use the function move_to
        response = move_to(pos, eef_orientation, move_mode, movement_duration, ignore_orientation, interpolation_type, check_limits)
    
        return response
    except rospy.ServiceException, e:
        print "MoveTo service call failed: %s"%e

# move to joints service
def moveToJointsClient(j0, j1, j2, interpolate, seconds):
    rospy.wait_for_service('uarm/move_to_joints')
    try:
        move_to_joints = rospy.ServiceProxy('uarm/move_to_joints', MoveToJoints)

        # Taken from the rqt_gui
        j3=float(0)
        move_mode = int(0) # absolute
        interpolation_type = interpolate
        check_limits = bool(True)
        # Builtin duration type from ROS
        # Duration(secs, nsecs)
        d = rospy.Duration(seconds,0)
        movement_duration = d
        # Use the function move_to
        response = move_to_joints(j0, j1, j2, j3, move_mode, movement_duration, interpolation_type, check_limits)
		
        return response
    except rospy.ServiceException, e:
        print "MoveToJoints service call failed: %s"%e

# pump service
def pumpClient(enable_pump):
    rospy.wait_for_service('uarm/pump')
    try:
        pump = rospy.ServiceProxy('uarm/pump', Pump)
        pump_status = enable_pump

        # Use the function pump
        response = pump(pump_status)

        return response
    except rospy.ServiceException, e:
        print "Pump service call failed: %s"%e

# Get orders from camera to grab/release or reset arm
def controlCallback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    if data.data == "reset":
        resetPosition() # reset arm
    elif data.data == "grab": # grab
        pumpClient(bool(True))
    elif data.data == "release": # release
        pumpClient(bool(False))

# get target and handle further actions
def targetCallback(data):
    rospy.loginfo("Target position: x: %f, y: %f, z: %f", data.x, data.y, data.z)

# def inverse_kinematics(x, y, z, check_limits=True):
#         L1=15
#         L2=23
# 	theta0=math.atan2(y,x) # j0
#         sqrtxy = math.sqrt(x*x + y*y)
#         theta2= math.pi - math.acos((x*x -y*y - L1*L1 + L2*L2 )/(2*L1*L2)) # j2
#         print (x*x +y*y+L1*L1 -L2*L2 )/(2*L1*sqrtxy)
#         theta1=math.acos((x*x +y*y+L1*L1 -L2*L2 )/(2*L1*math.sqrt(x*x+y*y))) - theta2 + math.pi/2 - math.atan2(y,x)
	
# 	#convert to degrees and offset origin
# 	theta0=theta0* 180/ math.pi - RESET_POS[0] # j0
# 	theta1=theta1 *( 180/ math.pi) - J_1_ZERO # j1
# 	theta2=(theta2* 180/ math.pi) - J_2_ZERO # j2


# 	return theta0,theta1,theta2

def inverse_kinematics(x, y, z, check_limits=True):
	L34 = L3 + L4
	sqrtxy2 = math.sqrt(x*x + y*y)
	
	if sqrtxy2 <= L34:
		beta = math.atan2(y,x)
		psi = math.acos((x*x + y*y + L3*L3 - L4*L4)/(2*L3*sqrtxy2))
		theta0 = beta # j0
		start = L1 * math.pi * 2 * theta0
		theta2 = math.acos((x*x + y*y + L3*L3 - L4*L4 )/(2*L3*L4)) # j2
    	if theta2 < 0:
    		theta1 = beta + psi
    	else:
    		theta1 = beta - psi # j1
	
		# Orientation of last link, phi, is the sum of all theta
		phi = theta0 + theta1 + theta2

		# Convert to degrees and offset origin
		theta0= theta0* (180/ math.pi) - start # j0
		theta1=theta1 *( 180/ math.pi) - J_1_ZERO # j1
		theta2=theta2* (180/ math.pi) - J_2_ZERO # j2
		phi =phi* (180/ math.pi)


	return theta0,theta1,theta2, phi

if __name__ == "__main__":
    # interpolation: 1: cubic. 2: linear. 0: none (very accurate).
    # use 1 second for large movements

    #resetPosition() # reset arm
    pumpClient(bool(False)) # disable pump

    rospy.init_node('arm_node', anonymous=True)
    rospy.Subscriber("uarm/target_position", Point, targetCallback) # get target position
    rospy.Subscriber("uarm/control", String, controlCallback) # grab, reset etc.
    j= inverse_kinematics(20, 10, 0)
    print j
    r=moveToJointsClient(j[0], j[1],j[2], 0, 0)
    if r.error: print("limits error")
rospy.spin()
    
    
    
    
