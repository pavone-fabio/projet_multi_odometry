import smbus
import time
import rospy
from std_msgs.msg import Int32

I2C_ADDR_ODOMETRY = 0x22

class I2CBus:
    def __init__(self, bus):
        self.bus = smbus.SMBus(bus)

    def read_byte_data(self, addr, reg):
        return self.bus.read_byte_data(addr, reg)

    def write_byte_data(self, addr, reg, data):
        self.bus.write_byte_data(addr, reg, data)

    def read_word_data(self, addr, reg):
        return self.bus.read_word_data(addr, reg)

    def write_word_data(self, addr, reg, data):
        self.bus.write_word_data(addr, reg, data)

class I2CDevice:
    def __init__(self, bus, addr):
        self.bus = bus
        self.addr = addr

    def read_register_byte(self, reg):
        return self.bus.read_byte_data(self.addr, reg)

    def read_register_word(self, reg):
        return self.bus.read_word_data(self.addr, reg)

    def read_register_multi(self, reg, count):
        return [self.bus.read_byte_data(self.addr, reg + i) for i in range(count)]

    def write_register_byte(self, reg, data):
        self.bus.write_byte_data(self.addr, reg, data)

    def write_register_word(self, reg, data):
        self.bus.write_word_data(self.addr, reg, data)

    def write_register_multi(self, reg, data):
        for i, value in enumerate(data):
            self.bus.write_byte_data(self.addr, reg + i, value)

class Odometry:
    def __init__(self, bus, addr):
        self.i2c = I2CDevice(bus, addr)
        self.robot_name = rospy.get_param('~robot_name', 'unknown_robot')
        self.odometry_pub = rospy.Publisher('/listener_2', Int32, queue_size=10)

    def read_pos(self):
        # Read position from the device
        position = self.i2c.read_register_word(0x22)
        #self.odometry_pub.publish(position)


def main():
    rospy.init_node('i2c_manager', anonymous=True)
    rospy.loginfo("i2c_manager is starting!")

    bus1 = I2CBus(1)

    odometry = Odometry(bus1, I2C_ADDR_ODOMETRY)

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        odometry.read_pos()
        rate.sleep()
        print(odometry.position)

if __name__ == '__main__':
    main()

