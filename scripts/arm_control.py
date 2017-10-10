#!/usr/bin/env python

import sys
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import String
from uarm.srv import *
import math
eef_x=float(0) # end effector position
eef_y=float(0)
eef_z=float(0)

# zero position for the joints
j0_zero=float(38)
j1_zero=float(130)
j2_zero=float(-7)

LOWER_LIMITS = [-45, -3.0, -24.0, -65.0]
UPPER_LIMITS = [115.0, 130.0, 90.0, 40.0]
J2_J1_MIN_LIMIT_OFFSET= 18.5
J2_J1_MAX_LIMIT_OFFSET = 153.0
NUM_JOINTS = 4
JOINT_NAMES = ['j0', 'j1', 'j2', 'j3']
CALIBRATION_CONFIG = [45.0, 130.0, -4.83, 0.0]
DEFAULT_EXECUTION_SLEEP_TIME = 0.1
MAXIMAL_NUM_INTERPOLATION_STEPS = 80.0
NO_INTERPOLATION = 'None'
LINEAR_INTERPOLATION = 'Linear'
CUBIC_INTERPOLATION = 'Cubic'
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
    resp= moveToJointsClient(j0_zero, j1_zero, j2_zero, int(1), 1)
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
        
        # Builtin duration type from ROS
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

def inverse_kinematics(x, y, z, check_limits=True):
    theta0=(math.atan2(y,x)* 180/ math.pi) + LOWER_LIMITS[0] # j0
    theta1=0 * 180/ math.pi # j1
    theta2= math.pi - math.acos((L1*L1 + L2*L2 - x*x -y*y)/(2*L1*L1 *L2*L2)) # j2
    theta2=(theta2* 180/ math.pi) + LOWER_LIMITS[2]

    return theta0,theta1,theta2

if __name__ == "__main__":
    # Change the x, y, z coordinates for the arm
    # This is where we can subscribe to the coordinates from
    # the camera
    # interpolation: 1: cubic. 2: linear. 0: none (very accurate).
    # use 1 second for large movements

    resetPosition() # reset arm
    #pumpClient(bool(False)) # disable pump

    rospy.init_node('arm_node', anonymous=True)
    rospy.Subscriber("uarm/target_position", Point, targetCallback) # get target position
    rospy.Subscriber("uarm/control", String, controlCallback) # grab, reset etc.
    print inverse_kinematics(10, 5, 0)
    rospy.spin()

