import rospy
from std_msgs.msg import String
import qwiic_vl53l1x
import time
import sys

def talker():
    pub = rospy.Publisher('distance', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1) # 10hz

    mySensor = qwiic_vl53l1x.QwiicVL53L1X()

    if mySensor._begin() == False:
        print("The Qwiic VL53L1X device isn't connected to the system. Please check your connection", file=sys.stderr)
        return

    mySensor.sensor_init()
    

    while not rospy.is_shutdown():
        try:
            
            mySensor.start_ranging()             # Write configuration bytes to initiate measurement
            time.sleep(0.005)
            distance = mySensor.get_distance()   # Get the result of the measurement from the sensor
            time.sleep(0.005)
            mySensor.stop_ranging()

            rospy.loginfo("Distance(mm): %s" % distance)
            pub.publish(str(distance))
            rate.sleep()

        except rospy.ROSInterruptException:
            pass
        except Exception as e:
            print(e)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
