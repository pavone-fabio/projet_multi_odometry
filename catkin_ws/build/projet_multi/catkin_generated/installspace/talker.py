#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def talker():
    # Initialize the ROS node
    rospy.init_node('talker', anonymous=True)
    
    # Create a publisher for the 'chatter' topic
    chatter_pub = rospy.Publisher('chatter', String, queue_size=10)
    
    # Set the loop rate
    rate = rospy.Rate(10)
    
    # Initialize the message count
    count = 0
    
    while not rospy.is_shutdown():
        # Create a new String message
        msg = String()
        
        # Fill the message with data
        msg.data = "hello world %s" % count
        
        # Log the message to the console
        rospy.loginfo(msg.data)
        
        # Publish the message
        chatter_pub.publish(msg)
        
        # Wait for the next iteration
        rate.sleep()
        count += 1

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
