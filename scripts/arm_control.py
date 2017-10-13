#!/usr/bin/env python

import sys
import rospy
from geometry_msgs.msg import Point
import uarm
from std_msgs.msg import String
from uarm.srv import *
import math
import ros_kth_uarm.kth_uarm as kth_uarm

hit = False
# get limits and calibration config from separate file
# /home/ras26/catkin_ws/src/ras_uarm/python/ros_kth_uarm
LOWER_LIMITS= kth_uarm.KTHUarm.LOWER_LIMITS
UPPER_LIMITS= kth_uarm.KTHUarm.UPPER_LIMITS
RESET_POS= kth_uarm.KTHUarm.CALIBRATION_CONFIG
J_0_ZERO= LOWER_LIMITS[0] # 90 degrees to the right
#J_1_ZERO=  77.0 # zero angle for j1
#J_2_ZERO= -32.0 # zero angle for j2
#J_1_ZERO= -30
J_1_ZERO= 2.5 # straight horisontal forward
J_2_ZERO= -23.3 # horisontal straight 
# LINKS
L1 = 10.645
L2 = 2.117
L3 = 14.825
L4 = 16.02
L5 = 3.3
L6 = 5.9

target_x = 0
target_y = 0
target_z = 0

recieved = False

# reset arm to neutral position
def resetPosition():
    print "Arm position reset..."
    resp= moveToJointsClient(RESET_POS[0], RESET_POS[1], RESET_POS[2], int(1), 1)
    if resp.error:
        print "Error, could not reset arm position. Check limits."


# move to joints service
def moveToJointsClient(j0, j1, j2, interpolate, seconds, move_mode=int(0)):
    rospy.wait_for_service('uarm/move_to_joints')
    try:
        move_to_joints = rospy.ServiceProxy('uarm/move_to_joints', MoveToJoints)

        move_mode = int(0) # absolute
        interpolation_type = interpolate
        check_limits = bool(True)
        # Builtin duration type from ROS
        # Duration(secs, nsecs)
        d = rospy.Duration(seconds,0)
        movement_duration = d
        # Use the function move_to
        j3=float(0.0)
        print "Sending to moveToJoints"
        print j0, j1, j2
        response = move_to_joints(j0, j1, j2, j3, move_mode, movement_duration, interpolation_type, check_limits)

        return response
    except rospy.ServiceException, e:
        print "MoveToJoints service call failed: %s"%e
        recieved = False

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
    global hit
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    if data.data == "reset":
        resetPosition() # reset arm
    elif data.data == "grab": # grab
        hit = bool(True)
        pumpClient(bool(True))

    elif data.data == "release": # release
        hit = bool(False)
        pumpClient(bool(False))

# get target and handle further actions
### Can't get the callback to get x, y, z
def targetCallback(msg):
    global recieved
    global target_x
    global target_y
    global target_z
    
    if (recieved == False):
        target_x = 100.0*msg.z + 10.5 + 1.5 - 3.5
        target_y = 100.0*msg.x
        target_z = -(100.0*msg.y + 11) + 3 + 8
        rospy.loginfo("Target position: x: %f, y: %f, z: %f", target_x, target_y, target_z)
        recieved = True
    
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

def inverse_kinematics(x, y, z):
    # set x y z directly
    print "begin with"
    print x, y, z
    L34 = L3 + L4

    lxy = math.sqrt(x*x + y*y)
    lxyz = math.sqrt(x*x + y*y + z*z)

    if lxy <= L34: # this means that the length of 
        beta = math.atan2(x, y)
        psi = math.acos( (lxyz*lxyz + L3*L3 - L4*L4)/(2*L3*lxyz) )
        omega = math.atan2(z, lxy)
        theta0 = beta  # j0
        theta1 = omega + psi #j1
        # start = L1 * math.pi * 2 * theta0
        eta = math.acos( (L3*L3 + L4*L4 - lxyz*lxyz)/(2*L3*L4) )
        print ("eta: " + str(eta/math.pi*180) + " omega " + str(omega/math.pi*180) + " psi " + str(psi/math.pi*180))
        theta2 = math.pi - eta - theta1 # j2

        print theta0* (180/ math.pi), theta1* (180/ math.pi), theta2* (180/ math.pi)
        # Orientation of last link, phi, is the sum of all theta
        phi = theta0 + theta1 + theta2  # j3

        # Convert to degrees and offset origin
        theta0= theta0 * (180/ math.pi) + J_0_ZERO #- RESET_POS[0]  # j0
        theta1=theta1 * ( 180/ math.pi) + J_1_ZERO  # j1
        theta2=theta2 * (180/ math.pi)  + J_2_ZERO  # j2
        phi =phi* (180/ math.pi)  # j3

    else:
        theta0=0
        theta1=0
        theta2=0
        print "position out of bounds"
    # theta0=0 + RESET_POS[0]
    return theta0, theta1, theta2


if __name__ == "__main__":

    # interpolation: 1: cubic. 2: linear. 0: none (very accurate).
    # use 1 second for large movements

    resetPosition() # reset arm
    pumpClient(bool(False)) # disable pump

    rospy.init_node('arm_node', anonymous=True)
    # Get coordinates of target position
    rospy.Subscriber("target_coord", Point, targetCallback)
 
    rospy.Subscriber("uarm/control", String, controlCallback) # grab, reset etc.
    
    target_x = 12
    target_y = -9
    target_z = -10
    
    # move above target, then pump, then down, then back
    j = inverse_kinematics(target_x, target_y, target_z)
    r = moveToJointsClient(j[0], j[1], j[2], 2 , 1) # above target
    pumpClient(bool(True)) # enable pump
    rel = inverse_kinematics(target_x, target_y, (target_z-5)) # move down in z
    r = moveToJointsClient(rel[0], rel[1],rel[2] , 0, 0) # movement down in z
    resetPosition()
    pumpClient(bool(False)) # disable pump
  	
    """
    k = -1.2
    while (hit==False):
        j1-=2
        j2+= -k
        if (hit==False):
            r = moveToJointsClient(RESET_POS[0], j1, j2, 0 , 0) # move down until hit
            if r.error: print("limits error")
        else:
            break
    
    resetPosition()
    """
    
    """
    while(not rospy.core.is_shutdown()):
        if ( recieved == True):
            recieved = False
            print "sending command"
            j = inverse_kinematics(target_x, target_y, target_z)
            r = moveToJointsClient(j[0], j[1], j[2], 0 , 0)
            print "sent and move"
            break
        rospy.rostime.wallsleep(1)
    """
    
    print "Done"
    rospy.spin()
    

