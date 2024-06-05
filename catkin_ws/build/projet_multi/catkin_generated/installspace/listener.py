#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def chatterCallback(msg):
    rospy.loginfo("I heard: [%s]", msg.data)

def listener():
    # Initialize the ROS node
    rospy.init_node('listener', anonymous=True)
    
    # Subscribe to the 'chatter' topic
    rospy.Subscriber('chatter', String, chatterCallback)
    
    # Spin() keeps Python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
