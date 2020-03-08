#!/usr/bin/env python

import copy
import time
import rospy
import numpy as np
from lab2_header import *
import sys

# 20Hz
SPIN_RATE = 20 

# UR3 home location
home = np.radians([120, -90, 90, -90, -90, 0])

# UR3 current position, using home position for initialization
current_position = copy.deepcopy(home)


############## Your Code Starts Here ##############


Q11 = np.radians([122.83, -54.38, 118.70, -154.29, -90.15, 3.23])
Q12 = np.radians([122.84, -63.04, 117.55, -144.48, -90.1, 3.2])
Q13 = np.radians([122.86, -69.41, 115.26, -135.82, -90.08, 3.17])
Q14 = np.radians([122.90, -79.8, 106.23, -116.4, -90.02, 3.09])
Q21 = np.radians([130.0, -49.11, 103.49, -144.31, -90.07, 10.38])
Q22 = np.radians([130.02, -54.84, 102.60, -137.69, -90.04, 10.35])
Q23 = np.radians([130.03, -60.26, 100.52, -130.19, -90.02, 10.32])
Q24 = np.radians([130.06, -69.26, 91.64, -112.32, -89.97, 10.23])
Q31 = np.radians([156.14, -58.85, 126.57, -157.49, -90.11, 36.55])
Q32 = np.radians([156.16, -66.69, 125.51, -148.58, -90.08, 36.52])
Q33 = np.radians([156.18, -74.11, 123.05, -138.7, -90.04, 36.49])
Q34 = np.radians([156.23, -85.84, 113.42, -117.35, -89.98, 36.43])
### Hint: How can you map this array to the towers?
Q = [ [Q11, Q12, Q13, Q14], \
      [Q21, Q22, Q23, Q24], \
      [Q31, Q32, Q33, Q34] ]












############### Your Code Ends Here ###############


thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

digital_in_0 = 0
analog_in_0 = 0

suction_on = True
suction_off = False

current_io_0 = False
current_position_set = False


############## Your Code Starts Here ##############

"""
TODO: define a ROS topic callback funtion for getting the state of suction cup
Whenever ur3/gripper_input publishes info this callback function is called.
"""

def suction_callback(msg):

	global digital_in_0
	digital_in_0 = msg.DIGIN
	# print("AIN0",msg.AIN0)
	# print("DIGIN", msg.DIGIN)









############### Your Code Ends Here ###############


"""
Whenever ur3/position publishes info, this callback function is called.
"""
def position_callback(msg):

	global thetas
	global current_position
	global current_position_set

	thetas[0] = msg.position[0]
	thetas[1] = msg.position[1]
	thetas[2] = msg.position[2]
	thetas[3] = msg.position[3]
	thetas[4] = msg.position[4]
	thetas[5] = msg.position[5]

	current_position[0] = thetas[0]
	current_position[1] = thetas[1]
	current_position[2] = thetas[2]
	current_position[3] = thetas[3]
	current_position[4] = thetas[4]
	current_position[5] = thetas[5]

	current_position_set = True


def gripper(pub_cmd, loop_rate, io_0):

	global SPIN_RATE
	global thetas
	global current_io_0
	global current_position

	error = 0
	spin_count = 0
	at_goal = 0

	current_io_0 = io_0

	driver_msg = command()
	driver_msg.destination = current_position
	driver_msg.v = 1.0
	driver_msg.a = 1.0
	driver_msg.io_0 = io_0   
	pub_cmd.publish(driver_msg)

	while(at_goal == 0):

		if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
			abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
			abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
			abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
			abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
			abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

			at_goal = 1
		
		loop_rate.sleep()

		if(spin_count >  SPIN_RATE*5):

			pub_cmd.publish(driver_msg)
			rospy.loginfo("Just published again driver_msg")
			spin_count = 0

		spin_count = spin_count + 1

	return error


def move_arm(pub_cmd, loop_rate, dest, vel, accel):

	global thetas
	global SPIN_RATE

	error = 0
	spin_count = 0
	at_goal = 0

	driver_msg = command()
	driver_msg.destination = dest
	driver_msg.v = vel
	driver_msg.a = accel
	driver_msg.io_0 = current_io_0
	pub_cmd.publish(driver_msg)

	loop_rate.sleep()

	while(at_goal == 0):

		if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
			abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
			abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
			abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
			abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
			abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

			at_goal = 1
			#rospy.loginfo("Goal is reached!")
		
		loop_rate.sleep()

		if(spin_count >  SPIN_RATE*5):

			pub_cmd.publish(driver_msg)
			rospy.loginfo("Just published again driver_msg")
			spin_count = 0

		spin_count = spin_count + 1

	return error


############## Your Code Starts Here ##############

def move_block(pub_command, loop_rate, Startpoint, Midpoint, \
	           Endpoint):
	global Q
	global analog_in_0
	### Hint: Use the Q array to map out your towers by location and height.

	move_arm(pub_command, loop_rate, home, 4.0, 4.0)

	move_arm(pub_command, loop_rate, Q[Startpoint][3], 4.0, 4.0)
	move_arm(pub_command, loop_rate, Q[Startpoint][2], 4.0, 4.0)
	gripper(pub_command, loop_rate, suction_on)	
	
	time.sleep(2.0)
	if(int(digital_in_0)==0):
		return 1
	move_arm(pub_command, loop_rate, Q[Startpoint][3], 4.0, 4.0)
	move_arm(pub_command, loop_rate, Q[Endpoint][3], 4.0, 4.0)
	move_arm(pub_command, loop_rate, Q[Endpoint][0], 4.0, 4.0)
	gripper(pub_command, loop_rate, suction_off)
	time.sleep(1.0)
	move_arm(pub_command, loop_rate, Q[Endpoint][3], 4.0, 4.0)
	move_arm(pub_command, loop_rate, Q[Startpoint][3], 4.0, 4.0)
	move_arm(pub_command, loop_rate, Q[Startpoint][3], 4.0, 4.0)
	move_arm(pub_command, loop_rate, Q[Startpoint][1], 4.0, 4.0)
	gripper(pub_command, loop_rate, suction_on)
	
	time.sleep(2.0)
	if(int(digital_in_0)==0):
		return 1
	move_arm(pub_command, loop_rate, Q[Startpoint][3], 4.0, 4.0)
	move_arm(pub_command, loop_rate, Q[Midpoint][3], 4.0, 4.0)
	move_arm(pub_command, loop_rate, Q[Midpoint][0], 4.0, 4.0)
	gripper(pub_command, loop_rate, suction_off)
	time.sleep(1.0)
	move_arm(pub_command, loop_rate, Q[Midpoint][3], 4.0, 4.0)
	move_arm(pub_command, loop_rate, Q[Endpoint][3], 4.0, 4.0)
	move_arm(pub_command, loop_rate, Q[Endpoint][0], 4.0, 4.0)
	gripper(pub_command, loop_rate, suction_on)
	
	time.sleep(2.0)
	if(int(digital_in_0)==0):
		return 1
	move_arm(pub_command, loop_rate, Q[Endpoint][3], 4.0, 4.0)
	move_arm(pub_command, loop_rate, Q[Midpoint][3], 4.0, 4.0)
	move_arm(pub_command, loop_rate, Q[Midpoint][1], 4.0, 4.0)
	gripper(pub_command, loop_rate, suction_off)
	time.sleep(1.0)
	move_arm(pub_command, loop_rate, Q[Midpoint][3], 4.0, 4.0)
	move_arm(pub_command, loop_rate, Q[Startpoint][3], 4.0, 4.0)
	move_arm(pub_command, loop_rate, Q[Startpoint][0], 4.0, 4.0)
	gripper(pub_command, loop_rate, suction_on)
	
	time.sleep(2.0)
	if(int(digital_in_0)==0):
		return 1
	move_arm(pub_command, loop_rate, Q[Startpoint][3], 4.0, 4.0)
	move_arm(pub_command, loop_rate, Q[Endpoint][3], 4.0, 4.0)
	move_arm(pub_command, loop_rate, Q[Endpoint][0], 4.0, 4.0)
	gripper(pub_command, loop_rate, suction_off)
	time.sleep(1.0)
	move_arm(pub_command, loop_rate, Q[Endpoint][3], 4.0, 4.0)
	move_arm(pub_command, loop_rate, Q[Midpoint][3], 4.0, 4.0)
	move_arm(pub_command, loop_rate, Q[Midpoint][1], 4.0, 4.0)
	gripper(pub_command, loop_rate, suction_on)
	
	time.sleep(2.0)
	if(int(digital_in_0)==0):
		return 1
	move_arm(pub_command, loop_rate, Q[Midpoint][3], 4.0, 4.0)
	move_arm(pub_command, loop_rate, Q[Startpoint][3], 4.0, 4.0)
	move_arm(pub_command, loop_rate, Q[Startpoint][0], 4.0, 4.0)
	gripper(pub_command, loop_rate, suction_off)
	time.sleep(1.0)
	move_arm(pub_command, loop_rate, Q[Startpoint][3], 4.0, 4.0)
	move_arm(pub_command, loop_rate, Q[Midpoint][3], 4.0, 4.0)
	move_arm(pub_command, loop_rate, Q[Midpoint][0], 4.0, 4.0)
	gripper(pub_command, loop_rate, suction_on)
	time.sleep(2.0)
	if(int(digital_in_0)==0):
		return 1
	move_arm(pub_command, loop_rate, Q[Midpoint][3], 4.0, 4.0)
	move_arm(pub_command, loop_rate, Q[Endpoint][3], 4.0, 4.0)
	move_arm(pub_command, loop_rate, Q[Endpoint][1], 4.0, 4.0)
	gripper(pub_command, loop_rate, suction_off)
	time.sleep(1.0)
	move_arm(pub_command, loop_rate, Q[Endpoint][3], 4.0, 4.0)
	move_arm(pub_command, loop_rate, Q[Startpoint][3], 4.0, 4.0)
	move_arm(pub_command, loop_rate, Q[Startpoint][0], 4.0, 4.0)
	gripper(pub_command, loop_rate, suction_on)
	time.sleep(2.0)
	if(int(digital_in_0)==0):
		return 1
	move_arm(pub_command, loop_rate, Q[Startpoint][3], 4.0, 4.0)
	move_arm(pub_command, loop_rate, Q[Endpoint][3], 4.0, 4.0)
	move_arm(pub_command, loop_rate, Q[Endpoint][2], 4.0, 4.0)
	gripper(pub_command, loop_rate, suction_off)
	time.sleep(1.0)
	move_arm(pub_command, loop_rate, home, 4.0, 4.0)
	
	error = 0

	return error



















############### Your Code Ends Here ###############


def main():

	global home
	global Q
	global SPIN_RATE

	# Initialize ROS node
	rospy.init_node('lab2node')

    # Initialize publisher for ur3/command with buffer size of 10
	pub_command = rospy.Publisher('ur3/command', command, queue_size=10)

	# Initialize subscriber to ur3/position and callback fuction
	# each time data is published
	sub_position = rospy.Subscriber('ur3/position', position, position_callback)

	############## Your Code Starts Here ##############
	# TODO: define a ROS subscriber for ur3/gripper_input message and corresponding callback function


	sub_gripper_input = rospy.Subscriber('ur3/gripper_input', gripper_input, suction_callback)





	############### Your Code Ends Here ###############


	############## Your Code Starts Here ##############
	# TODO: modify the code below so that program can get user input

	input_done = 0
	loop_count = 0
	Startpoint = 0
	Midpoint = 0
	Endpoint = 0



	while(not input_done):
		# input_string = raw_input("Enter number of loops <Either 1 2 3 or 0 to quit> ")
		# print("You entered " + input_string + "\n")

		input_string1 = raw_input("Enter Starting position <Either 1 2 3 or 0 to quit> ")
		print("You entered " + input_string1 + "\n")
		input_string2 = raw_input("Enter Ending position <Either 1 2 3 or 0 to quit> ")
		print("You entered " + input_string2 + "\n")

		# if(int(input_string) == 1):
		# 	input_done = 1
		# 	loop_count = 1
		# elif (int(input_string) == 2):
		# 	input_done = 1
		# 	loop_count = 2
		# elif (int(input_string) == 3):
		# 	input_done = 1
		# 	loop_count = 3
		if((int(input_string1)!=0) and (int(input_string2)!=0)):
			loop_count = 1
			input_done = 1
			Startpoint = int(input_string1)-1
			Endpoint = int(input_string2)-1
		elif (int(input_string) == 0):
			print("Quitting... ")
			sys.exit()
		else:
			print("Please just enter the character 1 2 3 or 0 to quit \n\n")

	for i in range(3):
		if (i!=Startpoint) and (i!=Endpoint):
			Midpoint = i
			break





	############### Your Code Ends Here ###############

	# Check if ROS is ready for operation
	while(rospy.is_shutdown()):
		print("ROS is shutdown!")

	rospy.loginfo("Sending Goals ...")

	loop_rate = rospy.Rate(SPIN_RATE)

	############## Your Code Starts Here ##############
	# TODO: modify the code so that UR3 can move a tower to a new location according to user input

	while(loop_count > 0):

		# move_arm(pub_command, loop_rate, home, 4.0, 4.0)

		# rospy.loginfo("Sending goal 1 ...")
		# move_arm(pub_command, loop_rate, Q[0][0], 4.0, 4.0)

		# gripper(pub_command, loop_rate, suction_on)
		# # Delay to make sure suction cup has grasped the block
		# time.sleep(1.0)

		# rospy.loginfo("Sending goal 2 ...")
		# move_arm(pub_command, loop_rate, Q[0][1], 4.0, 4.0)

		# rospy.loginfo("Sending goal 3 ...")
		# move_arm(pub_command, loop_rate, Q[0][2], 4.0, 4.0)

		
		error = move_block(pub_command, loop_rate, Startpoint, Midpoint, Endpoint)
		if error==1:
			print("Block is not where it should be... \n\n")
			gripper(pub_command, loop_rate, suction_off)
			time.sleep(1.0)
			move_arm(pub_command, loop_rate, home, 4.0, 4.0)
			print("Quitting...\n\n")
			sys.exit()
		loop_count = loop_count - 1

	gripper(pub_command, loop_rate, suction_off)










	############### Your Code Ends Here ###############



if __name__ == '__main__':
	
	try:
		main()
    # When Ctrl+C is executed, it catches the exception
	except rospy.ROSInterruptException:
		pass


	






