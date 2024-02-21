#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def talker():
    # The node will publish String messages to a topic named 'talking_topic'
    pub = rospy.Publisher('talking_topic', String, queue_size=10)
    rospy.init_node('talker_node', anonymous=True)
    rate = rospy.Rate(2) # 2hz

    while not rospy.is_shutdown(): # While the user has not pressed Ctrl+C
        hello_str = ": Hello ROS World %s" % rospy.get_time()
        print rospy.get_caller_id() + hello_str # Printing in the standard input
        pub.publish(hello_str) # Publishing message in the ROS topic 
        rate.sleep() # Wait an amount of time so that the loop rate will be as was set earlier (2hz)

    print 'Exiting node ' + rospy.get_name()

if __name__ == '__main__':
    talker()
