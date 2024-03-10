#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
from kobuki_msgs.msg import MotorPower

# Definition of class
class Collision_Detection_Node:
    def __init__(self, node_name):

        # Giving a name for the ROS node
        self.nname = node_name

        # ROS node initialization
        rospy.init_node(self.nname, anonymous=True)
        print 'Starting node ' + rospy.get_name()

        # Setting the rate/frequency by which the ROS loop will iterate
        self.rate = rospy.Rate(10) # 10hz

	# Declaring a publication to a ROS topic
        self.pub = rospy.Publisher('/mobile_base/commands/motor_power', MotorPower, queue_size=10)

	# ROS topic subscription
	imu_sub = rospy.Subscriber("/mobile_base/sensors/imu_data", Imu, self.imu_callback) 

	# Parameter
	self.force_current = {"x":0.0, "y":0.0, "z":0.0}	
	self.force_previous = {"x":0.0, "y":0.0, "z":0.0}
	self.is_ready = False

	rospy.spin() # Initiate the ROS loop
	


    def imu_callback(self, imu_info):
	# Retain old values
	self.force_previous = self.force_current.copy()
	# Get acceleration
	self.force_current["x"] = imu_info.linear_acceleration.x
	self.force_current["y"] = imu_info.linear_acceleration.y
	self.force_current["z"] = imu_info.linear_acceleration.z
	# Check if previous acceleration existed
	if self.is_ready:
	    self.verify()
	else:
	    self.is_ready = True        
		

    def verify(self):
	if not rospy.is_shutdown():
	    # Calculate the change in force
	    force_diff = {axis: self.force_current[axis] - self.force_previous[axis] for axis in self.force_current}  
	    #print abs(force_diff["x"]), abs(force_diff["y"]), abs(force_diff["z"])

	    # check if there is a collision
	    if any(abs(diff) >= 2 for diff in force_diff.values()):
	        motor_power = MotorPower()
	        motor_power.state = MotorPower.OFF
	        self.pub.publish(motor_power)
	    	# warnning
	        for axis, diff in force_diff.items():
		    if abs(diff) >= 2:
		        rospy.logwarn('There is a collision in {} direction!'.format(axis))
	    else:
	        motor_power = MotorPower()
	        motor_power.state = MotorPower.ON
	        self.pub.publish(motor_power)


if __name__ == '__main__':
    reactive_process_object = Collision_Detection_Node('Collision_Detection_Node')

