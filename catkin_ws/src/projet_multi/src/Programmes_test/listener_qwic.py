#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import qwiic_oled_display
import sys

myOLED = qwiic_oled_display.QwiicOledDisplay()

def callback(data):
    rospy.loginfo("Données reçues : %s", data.data)
    if myOLED.is_connected() == False:
        rospy.logerr("L'afficheur OLED Qwiic n'est pas connecté. Veuillez vérifier la connexion.")
        return

    myOLED.begin()
    myOLED.clear(myOLED.PAGE)
    myOLED.print(data.data)
    myOLED.display()

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('distance', String, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
