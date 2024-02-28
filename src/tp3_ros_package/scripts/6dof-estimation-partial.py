#!/usr/bin/env python
import rospy
import numpy as np
import sys
import sensor_msgs
import struct
import random
from sensor_msgs import point_cloud2
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import PointCloud2
import tf

# Definition of class
class Estimation_Node:
    def __init__(self, node_name):
    	self.nname = node_name     #Giving a name for the ROS node

    	rospy.init_node(self.nname, anonymous=True) #ROS node initialization

        self.num_of_plane_points = 100 # This sets a minimum number of points used to estimate a 3D plane

        self.plane_params = {"red":[-1]*4, "green":[-1]*4, "blue":[-1]*4} # A dictionnary holding the plane parameters, 4 per plane equation ax+by+cz+d = 0

        self.plane_points = {"red":[], "green":[], "blue":[]}

	self.feature_pose = Transform(Vector3(0, 0, 0.5), tf.transformations.quaternion_from_euler(0, 0, 0)) # This will hold the 6DOF pose of the feature, by a 3D vector for the translation and a quaternion for the rotation

	self.linear_solution = [] # This will hold the point of planes intersection obtained by solving a 3x3 linear system of equations

	point_cloud_sub = rospy.Subscriber("/camera/depth/points", PointCloud2, self.estimate_pose_callback) # ROS topic subscription

	self.br = tf.TransformBroadcaster()

	rospy.spin() # Initiate the ROS loop

    def empty_points(self):
	self.plane_points["red"] = []
	self.plane_points["green"] = []
	self.plane_points["blue"] = []

    def estimate_pose_callback(self, pointcloud_msg):
	print 'Received PointCloud2 message. Reading data...'
	point_list = sensor_msgs.point_cloud2.read_points(pointcloud_msg, skip_nans=True, field_names = ("x", "y", "z", "rgb"))

	print 'Retrieving coordinates and colors...'
	for point in point_list:
	    rgb = struct.unpack('BBBB', struct.pack('f', point[3]))

	    if rgb[2] > 100 and rgb[0] < 20 and rgb[1] < 20: # If dominant red point, concatenate it
	        self.plane_points["red"] += [[point[0], point[1], point[2]]]
	    elif rgb[1] > 100 and rgb[0] < 20 and rgb[2] < 20: # If dominant green point, concatenate it
	        self.plane_points["green"] += [[point[0], point[1], point[2]]]
	    elif rgb[0] > 100 and rgb[2] < 20 and rgb[1] < 20: # If dominant blue point, concatenate it
	        self.plane_points["blue"] += [[point[0], point[1], point[2]]]
	# Test if there are sufficient points for each plane
	### Enter your code ###
	if any(len(self.plane_points[color]) <= self.num_of_plane_points for color in ["red", "green", "blue"]):
    	    if len(self.plane_points["red"]) <= self.num_of_plane_points:
        	print 'There is not enough points for the red plane.'
    	    if len(self.plane_points["green"]) <= self.num_of_plane_points:
        	print 'There is not enough points for the green plane.'
    	    if len(self.plane_points["blue"]) <= self.num_of_plane_points:
        	print 'There is not enough points for the blue plane.'
	else:
	    # Estimate the plane equation for each colored point set using Least Squares algorithm
	    ### Enter your code ###
	    H_Red = np.array(self.plane_points["red"])
	    self.plane_params["red"] = np.append(-np.sum((np.linalg.inv(H_Red.T.dot(H_Red))).dot(H_Red.T),axis=1), 1)
	    H_Green = np.array(self.plane_points["green"])
	    self.plane_params["green"] = np.append(-np.sum((np.linalg.inv(H_Green.T.dot(H_Green))).dot(H_Green.T),axis=1), 1)    
	    H_Blue = np.array(self.plane_points["blue"])
	    self.plane_params["blue"] = np.append(-np.sum((np.linalg.inv(H_Blue.T.dot(H_Blue))).dot(H_Blue.T),axis=1), 1)    

	    # Verify that each pair of 3D planes are approximately orthogonal to each other
	    ### Enter your code ###

	    is_Orthgonal_RG = abs(np.sum(self.plane_params["red"][0:3]*self.plane_params["green"][0:3])) < 0.01
	    is_Orthgonal_GB = abs(np.sum(self.plane_params["green"][0:3]*self.plane_params["blue"][0:3])) < 0.01
	    is_Orthgonal_BR = abs(np.sum(self.plane_params["blue"][0:3]*self.plane_params["red"][0:3])) < 0.01
	    if not (is_Orthgonal_RG and is_Orthgonal_GB and is_Orthgonal_BR):
		print "Not all planes are orthogonal to each other. Will not estimate 6DOF pose."
	    # Feature detection
	    # Solve 3x3 linear system of equations given by the three intersecting planes, in order to find their point of intersection
	    ### Enter your code ###
	    else:
	        A = np.array([self.plane_params[color][0:3] for color in ["red", "green", "blue"]])
		b = np.array([-1,-1,-1]).T
		crossing_point = np.linalg.inv(A).dot(b)
	
	# Obtain z-axis (blue) vector as the vector orthogonal to the 3D plane defined by the red (x-axis) and the green (y-axis)
	### Enter your code ###
		z_axis = np.cross(self.plane_params["red"][0:3], self.plane_params["green"][0:3])
		z_axis /= np.linalg.norm(z_axis)

	# Obtain y-axis (green) vector as the vector orthogonal to the 3D plane defined by the blue (z-axis) and the red (x-axis)
	### Enter your code ###
		y_axis = np.cross(z_axis, self.plane_params["red"][0:3])
		y_axis /= np.linalg.norm(y_axis)
	# Construct the 3x3 rotation matrix whose columns correspond to the x, y and z axis respectively
	### Enter your code ###
		x_axis = self.plane_params["red"][0:3]
		x_axis /= np.linalg.norm(x_axis)
		R = np.array([x_axis, y_axis, z_axis]).T
		
	# Obtain the corresponding euler angles from the previous 3x3 rotation matrix
	### Enter your code ###
		sy = np.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])

		singular = sy < 1e-6

		if  not singular :
		    psi = np.arctan2(R[2,1] , R[2,2])
		    theta = np.arctan2(-R[2,0], sy)
		    phi = np.arctan2(R[1,0], R[0,0])
		else :
		    psi = np.arctan2(-R[1,2], R[1,1])
		    theta = np.arctan2(-R[2,0], sy)
		    phi = 0
		
		print psi
		print theta
		print phi

	# Set the translation part of the 6DOF pose 'self.feature_pose'
	### Enter your code ###

	# Set the rotation part of the 6DOF pose 'self.feature_pose'
	### Enter your code ###

    	# Publish the transform using the data stored in the 'self.feature_pose'
    	self.br.sendTransform((self.feature_pose.translation.x, self.feature_pose.translation.y, self.feature_pose.translation.z), 					self.feature_pose.rotation, rospy.Time.now(), "corner_6dof_pose", "camera_depth_optical_frame")

    	# Empty points
    	self.empty_points()

if __name__ == '__main__':
    my_estim_object = Estimation_Node('my_estimation_node')

