#!/usr/bin/env python
import serial, time
import rospy
import roslib
import math
import numpy as np
import config
import threading
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from listenerThread import ListenerThread

def main():
	ser = serial.Serial('/dev/ttyACM1', 115200)

	rospy.init_node('head_imu')

	listener = ListenerThread()
	listener.start()

	while 1:
	    serial_line = ser.readline()

	    split = serial_line.split('\t')

	    if(len(split) == 4):
	    	split[3] = split[3][:-2]

	    	config.head_yaw = float(split[1])
	    	config.head_pitch = float(split[2])
	    	config.head_roll = float(split[3])

	    	output = 'Yaw: {} Pitch: {} Roll: {}\n'.format(split[1], split[2], split[3])
	    	rospy.loginfo(output)

	    	publish_angles()


# Publish angles as ROS JointState message
def publish_angles():
	pub = rospy.Publisher('joint_states', JointState, queue_size=10)
	# Create new joint state message
	new_states = JointState()
	# Set header and timestamp
	new_states.header = Header()
	new_states.header.stamp = rospy.Time.now()
	# Set joint names
	new_states.name = [ 'waist_rotate', 'waist_lean', 'head_tilt', 'head_updown', 
						'head_leftright', 'jaw', 'eyes_updown', 'eye_leftright', 
						'left_eye_leftright', 'right_shoulder_up', 'right_bicep_rotate', 'right_bicep', 
						'right_shoulder_side', 'right_thumb1', 'right_thumb', 'right_thumb3', 
						'right_index1', 'right_index', 'right_index3', 'right_middle1', 
						'right_middle', 'right_middle3', 'right_ring1', 'right_ring', 
						'right_ring3', 'right_ring4', 'right_pinky1', 'right_pinky', 
						'right_pinky3', 'right_pinky4', 'right_hand', 'left_shoulder_up', 
						'left_bicep_rotate', 'left_bicep', 'left_shoulder_side', 'left_thumb1', 
						'left_thumb', 'left_thumb3', 'left_index1', 'left_index', 
						'left_index3', 'left_middle1', 'left_middle', 'left_middle3', 
						'left_ring1', 'left_ring', 'left_ring3', 'left_ring4', 
						'left_pinky1', 'left_pinky', 'left_pinky3', 'left_pinky4', 
						'left_hand']

	# Initialize joint positions
	new_states.position = np.zeros(53)

	# Set joint positions
	new_states.position[2] = (config.head_roll - config.roll_zero) * math.pi / 180
	new_states.position[3] = (config.head_pitch - config.pitch_zero) * math.pi / 180
	new_states.position[4] = (config.head_yaw - config.yaw_zero) * math.pi / 180
	new_states.position[11] = 0.985
	new_states.position[33] = 0.985

	# Set velocity and effort
	new_states.velocity = []
	new_states.effort = []

	# Publish the message
	pub.publish(new_states)



if __name__ == '__main__':
	main()
