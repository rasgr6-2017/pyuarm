#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState

j0 = 0
j1 = 0
j2 = 0

def callback(msg):
	global j0, j1, j2
	j0=msg.position[0]
	j1=msg.position[1]
	j2=msg.position[2]

if __name__ == "__main__":
	text_file = open("calibration_values.txt", "w")
	rospy.init_node('arm-calibration-node', anonymous=True)
	rospy.Subscriber("/uarm/joint_state", JointState, callback)
	print("---Calibrate-uArm Script---")
	
	raw_input("J0: Turn arm to the LEFT limit. Then Press ENTER.\n")
	text_file.write("%f\n" %j0)
	
	raw_input("J0: Turn arm to the RIGHT limit. Then Press ENTER.\n")
	text_file.write("%f\n" %j0)
	
	raw_input("J0: Turn arm to the ZERO position. Then Press ENTER.\n")
	text_file.write("%f\n" %j0)
	
	raw_input("J1: Turn link to the LOWEST position. Then Press ENTER.\n")
	text_file.write("%f\n" %j1)
	
	raw_input("J1: Turn link to the HIGHEST position. Then Press ENTER.\n")
	text_file.write("%f\n" %j1)
	
	raw_input("J1: Turn link to the ZERO position (Horisontal straight). Then Press ENTER.\n")
	text_file.write("%f\n" %j1)
	
	raw_input("J2: Turn link to the LOWEST position. Then Press ENTER.\n")
	text_file.write("%f\n" %j2)
	
	raw_input("J2: Turn link to the HIGHEST position. Then Press ENTER.\n")
	text_file.write("%f\n" %j2)
	
	raw_input("J2: Turn link to the ZERO position (Horisontal straight). Then Press ENTER.\n")
	text_file.write("%f\n" %j2)
	
	print("DONE!")
	
	
	text_file.close()
	
	
