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

        # Giving a name for the ROS node
        self.nname = node_name

        # ROS node initialization
        rospy.init_node(self.nname, anonymous=True)
        print 'Starting node ' + rospy.get_name()
        
        # Setting the rate/frequency by which the ROS loop will iterate
        self.rate = rospy.Rate(10) # 10hz
	
        # Declaring a publication to a ROS topic
        self.pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)
        

        
        # Parameters
        self.msg = Twist() # the message is send to the topic '/cmd_vel_mux/input/navi'
        self.key = None    # key is get from the keyboard
        self.coeff = 1     # Coefficient to modify the speed
        
        # Get paramaters from the .launch file
        self.Forward = rospy.get_param('Forward', 'u')              # Default to 'u' if not set
        self.Backward = rospy.get_param('Backward', 'j')            # Default to 'j' if not set
        self.RotateRight = rospy.get_param('RotateRight', 'k')      # Default to 'k' if not set
        self.RotateLeft = rospy.get_param('RotateLeft', 'h')        # Default to 'h' if not set
        self.IncreaseSpeed = rospy.get_param('IncreaseSpeed', 'f')  # Default to 'f' if not set
        self.DecreaseSpeed = rospy.get_param('DecreaseSpeed', 's')  # Default to 's' if not set

        # Run the program
        self.run()        
        
        print 'Exiting node ' + rospy.get_name()


    # This function get a pressed key on the keyboard to move the the TurtleBot2 inside Gazebo       
    def run(self):

	while not rospy.is_shutdown():
	
	    # Reset msg
	    self.msg.linear.x = 0
	    self.msg.angular.z = 0
	    
	    # get a pressed key on the keyboard
	    self.key = self.getKey()
	    
	    # Move forward when the 'u' button is pressed	    	
	    if (self.key == self.Forward):
	        print 'Key ' + self.Forward + ' is pressed.'
		self.msg.linear.x = 20*self.coeff # 20 [length unit]/[time unit]
		self.pub.publish(self.msg)             # publish to '/cmd_vel_mux/input/navi'
		
	    # Move backward when the 'j' button is pressed
	    elif (self.key == self.Backward):
	        print 'Key ' + self.Backward + ' is pressed.'
		self.msg.linear.x = -20*self.coeff # -20 [length unit]/[time unit]
		self.pub.publish(self.msg)              # publish to '/cmd_vel_mux/input/navi'
	    
	    # Move clockwise when the 'k' button is pressed
	    elif (self.key == self.RotateRight):
	        print 'Key ' + self.RotateRight + ' is pressed.'
		self.msg.angular.z = -30*self.coeff # -0.5236 rad/[time unit]
		self.pub.publish(self.msg)          # publish to '/cmd_vel_mux/input/navi'
	    
	    # Move counter-clockwise when the 'h' button is pressed
	    elif (self.key == self.RotateLeft):
	        print 'Key ' + self.RotateLeft + ' is pressed.' 
		self.msg.angular.z = 30*self.coeff # 0.5236  rad/[time unit]
		self.pub.publish(self.msg)         # publish to '/cmd_vel_mux/input/navi'

	    # Increase the linear and angular speed of the robot by 10% when the 'f' button is pressed
	    elif (self.key == self.IncreaseSpeed):
	    	print 'Key ' + self.IncreaseSpeed +' is pressed.'
		self.coeff = self.coeff*1.1

	    # Decrease the linear and angular speed of the robot by 10% when the 's' button is pressed
	    elif (self.key == self.DecreaseSpeed):
	    	print 'Key ', self.DecreaseSpeed + ' is pressed.'
		self.coeff = self.coeff*0.9
	    
	    # Exit loop when the 'e' button is pressed
	    elif (self.key == 'e'):
	    	print('Key e is pressed.')
		break

	    else:
		print('Invalid key.')
		
			
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
