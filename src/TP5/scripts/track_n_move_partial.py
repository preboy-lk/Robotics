#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
import cv2
import message_filters
import numpy as np
import matplotlib.pyplot as plt
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

	self.tracking_started = False
	self.track_window = None
	self.roi_hist = None

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
	try:
            x, y = np.int(centers[0][0]), np.int(centers[0][1])
            w, h = np.int(centers[1][0]- centers[0][0]), np.int(centers[1][1]-centers[0][1])
        except Exception as e:
            print("Only one Aruco in the field of view.\n ")
	    return img

	if not self.tracking_started:
	    # setup initial location of window
	    self.track_window = (x, y, w, h)
	    # set up the ROI for tracking
	    roi = img[y:y+h, x:x+w]
	    hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv_roi, np.array((0., 60.,32.)), np.array((180.,255.,255.)))
            self.roi_hist = cv2.calcHist([hsv_roi], [0], mask, [180], [0, 180])
	    cv2.normalize(self.roi_hist,self.roi_hist,0,255,cv2.NORM_MINMAX)
	    # Setup the termination criteria, either 15 iteration or move by at least 2 pt
	    self.term_crit = ( cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 15, 2 )
	    self.tracking_started = True
	else:
	    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	    dst = cv2.calcBackProject([hsv],[0],self.roi_hist,[0,180],1)
	    # apply meanshift to get the new location
	    ret, self.track_window = cv2.meanShift(dst, self.track_window, self.term_crit)
	    # Draw it on image
	    #print self.track_window
	    x,y,w,h = self.track_window
	img = cv2.rectangle(img, (x,y), (x+w,y+h), 255,2)
	cv2.imshow("Processed Image", img)
	cv2.waitKey(1)

        # Publish commands
        linear = 0.
        angular = 0.
        if ((x+w/2 - img.shape[1]/2) > 50):
	    angular = -0.2
        elif ((img.shape[1]/2 - (x+w/2)) > 50):
	    angular = 0.2
        else:
   	    if (depth[np.round(x+w/2)][np.round(y+h/2)] > 0.5):
	        linear = 0.2
        print 'Jokers found. Start tracking'
       	self.send_commands(linear, angular)        
        return img

    def process_image_circle(self, img, depth):
        """
        Processing of an image with a circle.
        :param img: opencv image
        :return img: processed opencv image
        """
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
	# Draw detected circles on the original image
	if circles is not None:
	    circles = np.uint16(np.around(circles))
	    for i in circles[0,:]:
		 # draw the outer circle
	        cv2.circle(filtered_img,(i[0],i[1]),i[2],(0,255,0),2)
		 # draw the center of the circle
		cv2.circle(filtered_img,(i[0],i[1]),2,(0,0,255),3)
	self.image_pub.publish(self.bridge.cv2_to_imgmsg(filtered_img, encoding = 'passthrough'))
	cv2.imshow("Processed Image", filtered_img)
	cv2.waitKey(1)
	
	# Publish necessary commands
        linear = 0.
        angular = 0.
	if circles is None:
	    angular = 0.4
	    print 'Circle not found. Rotate to find circle'
	else:
	    max_radius = 0
	    center = []
	    # This code to ensure we take the circle with biggest radius
	    for i in circles[0,:]:
		if i[2] > max_radius: 
		    max_radius = i[2]
		    center = i
	    if ((center[0] - img.shape[1]/2) > 50):
		angular = -0.2
	    elif ((img.shape[1]/2 - center[0]) > 50):
		angular = 0.2
	    else:
	    	if (depth[center[0]][center[1]] > 1):
		    linear = 0.2
	    print 'Circle found. Start tracking'
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

