#!/usr/bin/env python
import rospy
import sys
from sensor_msgs.msg import JointState
from uarm.srv import *

j0 = 0
j1 = 0
j2 = 0

# detach/attach service
def servoClient(attach):
    rospy.wait_for_service('uarm/attach_servos')
    try:
        servos = rospy.ServiceProxy('uarm/attach_servos', AttachDetach)
        response = servos(attach)
        return response
    except rospy.ServiceException, e:
        print "attach/detach servos service call failed: %s"%e


def callback(msg):
	global j0, j1, j2
	j0=msg.position[0]
	j1=msg.position[1]
	j2=msg.position[2]

def main():
	text_file = open("calibration_values.txt", "w")
	rospy.init_node('arm_calibration_node', anonymous=True)
	rospy.Subscriber("/uarm/joint_state", JointState, callback)
	servoClient(False)
	print("---Calibrate-uArm Script---\nMake sure it has power supply! Robot should face forward from you.")
		
	raw_input("J0: Turn arm to the RIGHT limit. Then Press ENTER.\n") # j0 Upper limit
	print j0
	text_file.write("%f\n" %j0)
	
	raw_input("J0: Turn arm to the LEFT limit. Then Press ENTER.\n") # j0 lower limit
	print j0
	text_file.write("%f\n" %j0)

	raw_input("J0: Turn arm to the ZERO (middle) position. Then Press ENTER.\n") # j0 zero and reset position
	print j0
	text_file.write("%f\n" %j0)

	raw_input("J1: Turn link to the LOWEST (down) position. Then Press ENTER.\n") # j1 lower limit
	print j1
	text_file.write("%f\n" %j1)
	
	raw_input("J1: Turn link to the HIGHEST (up-back) position. Then Press ENTER.\n") # j1 upper limit
	print j1
	text_file.write("%f\n" %j1)
	
	raw_input("J1: Turn link to the ZERO position (Planar Horisontal straight forward). Then Press ENTER.\n") # j1 zero origin
	print j1
	text_file.write("%f\n" %j1)
	
	raw_input("J2: Turn link to the LOWEST (back) position. Then Press ENTER.\n") # j2 lower limit
	print j2
	text_file.write("%f\n" %j2)
	
	raw_input("J2: Turn link to the HIGHEST (forward-down) position. Then Press ENTER.\n") # j2 upper limit
	print j2
	text_file.write("%f\n" %j2)
	
	raw_input("J2: Turn link to the ZERO position (Planar Horisontal straight). Then Press ENTER.\n") # j2 zero origin position
	print j2
	text_file.write("%f\n" %j2)
	
	servoClient(True)
	text_file.close()
	print("DONE!")
	rospy.signal_shutdown("done")
	
	
if __name__ == "__main__":
	main()

	
	
