import smbus
import time
import rospy
from std_msgs.msg import Int32

I2C_ADDR_ODOMETRY = 0x22

REG_COUNT_START = 0x01
REG_COUNT_STOP = 0x02
REG_COUNT_VALUE = 0x31

def main():
    rospy.init_node('i2c_manager', anonymous=True)
    rospy.loginfo("i2c_manager is starting!")

    # Configuration du bus I2C
    bus = smbus.SMBus(1)

    # Démarrage du comptage
    bus.write_byte_data(I2C_ADDR_ODOMETRY, REG_COUNT_START, 0x01)

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        # Lecture de la valeur de comptage
        count_high = bus.read_byte_data(I2C_ADDR_ODOMETRY, REG_COUNT_VALUE)
        count_low = bus.read_byte_data(I2C_ADDR_ODOMETRY, REG_COUNT_VALUE + 1)
        count = (count_high << 8) | count_low
        rospy.loginfo("Count: %d" % count)
        rate.sleep()

    # Arrêt du comptage
    bus.write_byte_data(I2C_ADDR_ODOMETRY, REG_COUNT_STOP, 0x01)

if __name__ == '__main__':
    main()

