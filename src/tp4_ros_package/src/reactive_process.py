#!/usr/bin/env python
import rospy
import numpy as np
import sys
import sensor_msgs
import struct
import random
from sensor_msgs.msg import Imu

import tf

# Definition of class
class Estimation_Node:
    def __init__(self, node_name):
    	self.nname = node_name     #Giving a name for the ROS node

    	rospy.init_node(self.nname, anonymous=True) #ROS node initialization

	imu_sub = rospy.Subscriber("/mobile_base/sensors/imu_data", Imu, self.collision_detect_callback) # ROS topic subscription
	self.position = [0,0,0]	
	rospy.spin() # Initiate the ROS loop

    def collision_detect_callback(self, imu_info):
	x = imu_info.linear_acceleration.x
	y = imu_info.linear_acceleration.y
	z = imu_info.linear_acceleration.z
	print z


if __name__ == '__main__':
    reactive_process_object = Estimation_Node('reactive_process')

