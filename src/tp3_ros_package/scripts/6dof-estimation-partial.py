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

	# A dictionnary holding the plane parameters, 4 per plane equation ax+by+cz+d = 0
        self.plane_params = {"red":[-1]*4, "green":[-1]*4, "blue":[-1]*4} 

        self.plane_points = {"red":[], "green":[], "blue":[]}

	self.colors = list(self.plane_points.keys()) # Names of planes

	# This will hold the 6DOF pose of the feature, by a 3D vector for the translation and a quaternion for the rotation
	self.feature_pose = Transform(Vector3(0, 0, 0.5), tf.transformations.quaternion_from_euler(0, 0, 0)) 

	# This will hold the point of planes intersection obtained by solving a 3x3 linear system of equations
	self.linear_solution = [] 

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
	# If the number of points belonging to any planes is inferior than minimum number of points (100 points),
	# the notification will be shown in the terminal.		
	if any(len(self.plane_points[color]) <= self.num_of_plane_points for color in ["red", "green", "blue"]):
    	    if len(self.plane_points["red"]) <= self.num_of_plane_points:
		print 'There is not enough points for the red plane.'
    	    if len(self.plane_points["green"]) <= self.num_of_plane_points:
		print 'There is not enough points for the green plane.'
    	    if len(self.plane_points["blue"]) <= self.num_of_plane_points:
		print 'There is not enough points for the blue plane.'
	else:
	    # Estimate the plane equation for each colored point set using Least Squares algorithm
	    for color in self.colors:

		# H is matrix containing all points in the current color plane. H size is (number of points,3) 
		H = np.array(self.plane_points[color])  

		# Matrix 1 with size (number of points,1) due to set plane equation ax+by+cz = 1 with d = -1
		y = np.array([1] * len(self.plane_points[color]))  

		self.plane_params[color][0:3] = np.dot(np.dot(np.linalg.inv(H.T.dot(H)), H.T), y) # Solution of LS: (H'*H)^(-1)*H'*y

		# Verify that each pair of 3D planes are approximately orthogonal to each other
		# Two planes are orthogonal if dot product of their normal vector are 0. Set threshold is 0.1 due to the calculation error. 
		is_Orthgonal_RG = abs(np.dot(self.plane_params["red"][0:3],self.plane_params["green"][0:3])) < 0.1
		is_Orthgonal_GB = abs(np.dot(self.plane_params["green"][0:3],self.plane_params["blue"][0:3])) < 0.1
		is_Orthgonal_BR = abs(np.dot(self.plane_params["blue"][0:3],self.plane_params["red"][0:3])) < 0.1
		if not (is_Orthgonal_RG and is_Orthgonal_GB and is_Orthgonal_BR):
		    print "Not all planes are orthogonal to each other. Will not estimate 6DOF pose." # Notification in the terminal if not orthogonal
		else:
		    # Feature detection
		    # Solve 3x3 linear system of equations given by the three intersecting planes, in order to find their point of intersection

		    # A is matrix containing planes parameter. A size is (number of planes,3)
		    A = np.array([self.plane_params[color][0:3] for color in ["red", "green", "blue"]])
		    
		    # Matrix 1 with size (Dimension of points,1) due to set plane equation ax+by+cz = 1 with d = -1
		    b = np.array([1,1,1]).T

		    self.linear_solution = np.linalg.inv(A).dot(b) # Solution: A^(-1)*b

		    # Obtain z-axis (blue) vector as the vector orthogonal to the 3D plane defined by the red (x-axis) and the green (y-axis)
		    z_axis = np.cross(self.plane_params["red"][0:3], self.plane_params["green"][0:3])
		    z_axis /= np.linalg.norm(z_axis) # Normalization

		    # Obtain y-axis (green) vector as the vector orthogonal to the 3D plane defined by the blue (z-axis) and the red (x-axis)
		    y_axis = np.cross(self.plane_params["blue"][0:3], self.plane_params["red"][0:3])
		    y_axis /= np.linalg.norm(y_axis) # Normalization

		    # Obtain x-axis (red) vector as the vector orthogonal to the 3D plane defined by the green (y-axis) and the blue (z-axis)
		    x_axis = np.cross(self.plane_params["green"][0:3], self.plane_params["blue"][0:3])
		    x_axis /= np.linalg.norm(x_axis) # Normalization

		    # Construct the 3x3 rotation matrix whose columns correspond to the x, y and z axis respectively
		    R = np.array([x_axis, y_axis, z_axis]).T
	
		    # Obtain the corresponding euler angles from the previous 3x3 rotation matrix
		    # (psi, theta, phi) represent rotations around the axes (Z, Y, X)
		    if (R[2,0] != 1 and R[2,0] != -1):  # No Gimbal Lock
			theta = -np.arcsin(R[2,0])
			psi = np.arctan2(R[2,1]/np.cos(theta), R[2,2]/np.cos(theta))
			phi = np.arctan2(R[1,0]/np.cos(theta), R[0,0]/np.cos(theta))
		    else : # Gimbal Lock
			phi = 0 # Can set to anything
			if (R[2,0] == -1) :
				theta = np.pi/2
				psi = phi + np.arctan2(R[0,1], R[0,2])
			else:
				theta = -np.pi/2
				psi = -phi + np.arctan2(-R[0,1], -R[0,2])

		    # Set the translation part of the 6DOF pose 'self.feature_pose'
		    self.feature_pose.translation.x = self.linear_solution[0] # x of the point of intersection
		    self.feature_pose.translation.y = self.linear_solution[1] # y of the point of intersection
		    self.feature_pose.translation.z = self.linear_solution[2] # z of the point of intersection

		    # Set the rotation part of the 6DOF pose 'self.feature_pose'
		    rotation_quaternion = tf.transformations.quaternion_from_euler(psi, theta, phi) # Convert Euler angles to a quaternion
		    self.feature_pose.rotation = rotation_quaternion

		    # Publish the transform using the data stored in the 'self.feature_pose'
		    self.br.sendTransform((self.feature_pose.translation.x, self.feature_pose.translation.y, self.feature_pose.translation.z),self.feature_pose.rotation, rospy.Time.now(), "corner_6dof_pose", "camera_depth_optical_frame")

	# Empty points
	self.empty_points()

if __name__ == '__main__':
    my_estim_object = Estimation_Node('my_estimation_node')

