#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
import cv2
import message_filters
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

"""
Hints:
    Aruco markers:
        https://mecaruco2.readthedocs.io/en/latest/notebooks_rst/Aruco/Aruco.html
    Histogram Backprojection
        https://docs.opencv.org/3.4/dc/df6/tutorial_py_histogram_backprojection.html
    Meanshift & Camshift:
        https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_video/py_meanshift/py_meanshift.html
"""

class Node:

    def __init__(self, type_):
        """
        Initialization of aproximated synchronized subscription to topics.
        """
        self.mode = type_
        rospy.init_node('track_n_move')
        # Topic subscription
        rgb_sub = message_filters.Subscriber('/camera/rgb/image_raw', Image)
        depth_sub = message_filters.Subscriber('/camera/depth/image_raw', Image)

        # Synchronization of received messages
        ats = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub], queue_size=5, slop=0.1)
        ats.registerCallback(self.callback)
        # ts = message_filters.TimeSynchronizer([rgb_sub, depth_sub], 1)
        # ts.registerCallback(self.callback)

        # OpenCV bridge to convert ROS image into OpenCV image
        self.bridge = CvBridge()

        # Publisher of a velocity commands        
        self.pub_vel = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=1)
        # Publisher of a processed image
        self.image_pub = rospy.Publisher('/processed_image', Image, queue_size=1)
        
        # Creating aruco_dict with 5x5 bits with max 250 ids
        # Ids ranges from 0-249
        self.parameters = cv2.aruco.DetectorParameters_create()
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_250)

        # Velocity control message
        self.msg_vel = Twist()
        rospy.spin()

    def detect_aruco(self, img):
        """
        Wrapper around  OpenCV detectMarkers function
        Nothing to do here.
        :param img: cv image
        :return: list(dict(id: list(corners),...))
        """
        aruco_list = {}
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # lists of ids and the corners beloning to each id
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
        if len(corners):
            for k in range(len(corners)):
                temp_1 = corners[k]
                temp_1 = temp_1[0]
                temp_2 = ids[k]
                temp_2 = temp_2[0]
                aruco_list[temp_2] = temp_1
            return aruco_list

    def mark_Aruco(self, img, aruco_list):
        """
        Nothing to do here.
        :param img: opencv image
        :param aruco_list: list of detected aruco markers
        :return img: opencv image with drawn marker centers
        :return centers: centers of aruco markers
        """
        key_list = aruco_list.keys()
        centers = []
        font = cv2.FONT_HERSHEY_SIMPLEX
        for key in key_list:
            dict_entry = aruco_list[key]    
            centre = dict_entry[0] + dict_entry[1] + dict_entry[2] + dict_entry[3]
            centre[:] = [int(x / 4) for x in centre] 
            orient_centre = centre + [0.0,5.0]
            centre = tuple(centre)  
            orient_centre = tuple((dict_entry[0]+dict_entry[1])/2)
            centers.append(list(centre))
            cv2.circle(img,centre,1,(0,0,255),8)
            cv2.line(img,centre,orient_centre,(255,0,0),4) 
        return img, centers

    def callback(self, rgb, depth):
        """
        This transforms a ROS image into an OpenCV image, 
        calls for a corresponding processing method and publishes a processed ROS image.
        Nothing to do here.
        :param data: ROS RGB and depth images
        """
        try:
            cv_rgb_image = self.bridge.imgmsg_to_cv2(rgb, "bgr8")
            cv_depth_image = self.bridge.imgmsg_to_cv2(depth, "32FC1")
        except CvBridgeError as e:
            rospy.loginfo('ROS img -> cv2 img conversion failure')
            return None

        if self.mode == "circle":
            # Part I: red circle
            cv_image = self.process_image_circle(cv_rgb_image, cv_depth_image)
        elif self.mode == "joker":
            # Part II: a random image
            cv_image = self.process_random_image(cv_rgb_image, cv_depth_image)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            rospy.loginfo('cv2 img -> ROS img conversion failure')

    def process_random_image(self, img, depth):
        """
        Processing of image with a random picture
        Hint 1: maybe, you will need to add a source of light in Gazebo.
                It is on the top panel.
        Hint 2: probably, it is worth to move the robot somehow. 
        :param img: opencv image
        :return img: processed opencv image
        """
        try:
            # Detect Aruco markers
            aruco_list = self.detect_aruco(img)
            # Draw their centers and find their centers
            img, centers = self.mark_Aruco(img, aruco_list)
        except Exception as e:
            print("No Aruco markers in the field of view.\n")
            return img

        # Your code starts here
        # ...

        # Publish commands
        linear = 0.
        angular = 0.
        self.send_commands(linear, angular)        
        return img

    def process_image_circle(self, img, depth):
        """
        Processing of an image with a circle.
        :param img: opencv image
        :return img: processed opencv image
        """
	while not rospy.is_shutdown():
	    # Circle detection
	    # Your code starts here
	    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	    # define range of blue color in HSV
	    lower_red = np.array([0,100,100])
	    upper_red = np.array([10,255,255])
	    # Threshold the HSV image to get only blue colors
	    mask = cv2.inRange(hsv, lower_red, upper_red)
	    # Bitwise-AND mask and original image
	    filtered_img = cv2.bitwise_and(img,img, mask= mask)
	    gray_img = cv2.cvtColor(filtered_img, cv2.COLOR_BGR2GRAY)
	    blurred_img = cv2.medianBlur(gray_img,5)
	    circles = cv2.HoughCircles(blurred_img,cv2.HOUGH_GRADIENT,1,filtered_img.shape[0]*2, param1=300, param2=1)
	    center = [0,0]
	    #print circles
            # Draw detected circles on the original image
            if circles is not None:
		circles = np.uint16(np.around(circles))
		for i in circles[0,:]:
		 # draw the outer circle
		    cv2.circle(filtered_img,(i[0],i[1]),i[2],(0,255,0),2)
		 # draw the center of the circle
		    center[0] = i[0]
		    center[1] = i[1]
		    cv2.circle(filtered_img,(i[0],i[1]),2,(0,0,255),3)
	    
	    #if np.shape(center) == 2:
	    print center[0], center[1]
	    print depth[center[0]][center[1]]
	    self.image_pub.publish(self.bridge.cv2_to_imgmsg(filtered_img, encoding = 'passthrough'))
	    cv2.imshow("Processed Image", filtered_img)
       	    cv2.waitKey(1)
	# Publish necessary commands
        linear = 0.
        angular = 0.
        self.send_commands(linear, angular)        
        return img
	
        
    def send_commands(self, linear, angular):
        """
        Nothing to do here.
        :param cmds: dictionary of velocity commands
        """
        self.msg_vel.linear.x = linear
        self.msg_vel.angular.z = angular
        self.pub_vel.publish(self.msg_vel)


if __name__ == '__main__':
    if len(sys.argv) == 1:
        print("Looking for a circle")
        Node("circle")
    else:
        print("Looking for a {}".format(sys.argv[1]))
        Node(sys.argv[1])

