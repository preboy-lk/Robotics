#!/usr/bin/env python
import rospy
from std_msgs.msg import String

# Function which is called when a ROS message is heard in the respective ROS topic (see Subscriber initialization further down). The message is stored in the function's argument 'data'
def callback(data):
    print rospy.get_caller_id() + ": I heard " + data.data
    
def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off.
    rospy.init_node('listener_node', anonymous=True)

    # The node will subscribe to a ROS topic named 'talking_topic' and whenever a ROS message is heard there, the 'callback' function will be executed. 
    rospy.Subscriber("talking_topic", String, callback)

    # rospy.spin() keeps python from exiting until the node is stopped (by pressing Ctrl+c for example). Also, by entering this infinite loop, it allows any ROS callback functions to be executed, whenever a message is listened from the respective ROS topic. The variant function 'rospy.spinOnce()' would allow the callbacks to be executed only once and after exit.
    rospy.spin()

    print 'Exiting node ' + rospy.get_name() # This will only be executed if 'rospy.spin()' finishes, after having pressed Ctrl+c

if __name__ == '__main__':
    listener()
    
