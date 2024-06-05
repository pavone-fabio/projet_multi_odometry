#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import smbus
import time

I2C_ADDR_ODOMETRY = 0x22

bus = smbus.SMBus(1)  # Rev 2 Pi uses 1

def talker():
    # Initialize the ROS node
    rospy.init_node('talker', anonymous=True)
    
    # Create a publisher for the 'chatter' topic
    chatter_pub = rospy.Publisher('chatter', String, queue_size=10)
    
    # Set the loop rate
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        # Read the value from the slave I2C
        value = bus.read_word_data(I2C_ADDR_ODOMETRY, 0x31)
        
        # Create a new String message
        msg = String()
        
        # Fill the message with data
        msg.data = "Slave value: {:.2f} m".format(value)
        
        # Log the message to the console
        rospy.loginfo(msg.data)
        
        # Publish the message
        chatter_pub.publish(msg)
        
        # Wait for the next iteration
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

