#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
import numpy
from cv_bridge import CvBridge, CvBridgeError

def callback_rgb(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard an rgb image")
    bridge = CvBridge()
    try:
      #Transform image type sensor_msgs/Image of ROS to a classical OpenCV image
      cv_image_rgb = bridge.imgmsg_to_cv2(data, "bgr8")
      #At this moment, we can process cv_image_rgb as a classical 3 channels OpenCV image
    except CvBridgeError as e:
      print(e)

    #Design a blue circle in the image
    cv2.circle(cv_image_rgb, (50,50), 10, (255,0,0))

    cv2.imshow("Image window for rgb image", cv_image_rgb)
    cv2.waitKey(3)

def callback_depth(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard a depth image")
    bridge = CvBridge()
    try:
      #Transform image type sensor_msgs/Image of ROS to a classical OpenCV image
      cv_image_depth = bridge.imgmsg_to_cv2(data, "passthrough")
       #At this moment, we can process cv_image_depth as a classical 1 channel OpenCV image

      # Normalize the depth image to fall between 0 (black) and 1 (white)
      cv2.normalize(cv_image_depth, cv_image_depth, 0, 1, cv2.NORM_MINMAX) 

    except CvBridgeError as e:
      print(e)

    #Design a white circle in the image
    cv2.circle(cv_image_depth, (50,50), 10, 255)

    #cv2.imshow("Image window for depth image", cv_image_depth)
    #cv2.waitKey(3)

def listener():
    rospy.init_node('image_node', anonymous=True)

    rospy.Subscriber("/camera/rgb/image_raw", Image, callback_rgb)
    rospy.Subscriber("/camera/depth/image_raw", Image, callback_depth)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
