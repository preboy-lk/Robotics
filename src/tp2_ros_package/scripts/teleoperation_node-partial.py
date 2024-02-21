#!/usr/bin/env python
import rospy
import numpy as np
import tty
import sys
import select
import termios
from geometry_msgs.msg import Twist

# Definition of class
class Teleoperation_Node:

    def __init__(self, node_name):
	#############
	# YOUR CODE #
	#############
	pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)
    	rospy.init_node(node_name, anonymous=True)
    	rate = rospy.Rate(2) # 2hz
	step_linear = 100
	step_angular = 30
	coeff_step = 1
	msg = Twist()
	key = self.getKey()
	Forward = rospy.get_param('Forward', 'default_value')
	Backward = rospy.get_param('Backward', 'default_value')
	RotateRight = rospy.get_param('RotateRight', 'default_value')
	RotateLeft = rospy.get_param('RotateLeft', 'default_value')
	IncreaseSpeed = rospy.get_param('IncreaseSpeed', 'default_value')
	DecreaseSpeed = rospy.get_param('decreaseSpeed', 'default_value')

	while not rospy.is_shutdown():
	    msg.linear.x = 0
	    msg.angular.z = 0
	    if (self.getKey() == 'f'):
		coeff_step *= 1.1

	    elif (self.getKey() == 's'):
		coeff_step *= 0.9
   	
	    elif (self.getKey() == 'u'):
		msg.linear.x = coeff_step*step_linear
	        pub.publish(msg)
		
	    elif (self.getKey() == 'j'):
		msg.linear.x = -coeff_step*step_linear
	        pub.publish(msg)
		
	    elif (self.getKey() == 'k'):
		msg.angular.z = -coeff_step*step_angular #-0.5236
	        pub.publish(msg)
		
	    elif (self.getKey() == 'h'):
		msg.angular.z = coeff_step*step_angular #0.5236
	        pub.publish(msg)

	    elif (self.getKey() == 'e'):
		break

	    else:
		print('Invalid key.')	
	rate.sleep()
	print 'Exiting node ' + rospy.get_name()

    # This function reads a single keyboard character from the terminal and returns this character
    def getKey(self):
	# Back-up default terminal settings
	settings = termios.tcgetattr(sys.stdin)

        tty.setraw(sys.stdin.fileno()) # Setting stdio terminal to raw (no need for pressing enter)
        key = sys.stdin.read(1) # Read 1 character 
	
	# Restore default terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key



if __name__ == '__main__':
    myteleopobject = Teleoperation_Node('my_teleoperation_node')
