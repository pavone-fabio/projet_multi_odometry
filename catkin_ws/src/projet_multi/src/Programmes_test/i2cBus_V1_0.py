import smbus
import time
import threading
import rospy

I2C_ADDR_ODOMETRY = 0x22
I2C_ADDR_TIRETTE = 0x08
I2C_ADDR_DRIVER_BRUSHLESS = 0x17
I2C_ADDR_CAKES_COLOR_MUX = 0x70
I2C_ADDR_CAKES_COLOR_SENSOR = 0x29
I2C_ADDR_LEDS = 0x20

class I2CBus:
    def __init__(self, bus):
        self.bus = smbus.SMBus(bus)
        self.lock = threading.Lock()

    def read_byte_data(self, addr, reg):
        with self.lock:
            return self.bus.read_byte_data(addr, reg)

    def write_byte_data(self, addr, reg, data):
        with self.lock:
            self.bus.write_byte_data(addr, reg, data)

    def read_word_data(self, addr, reg):
        with self.lock:
            return self.bus.read_word_data(addr, reg)

    def write_word_data(self, addr, reg, data):
        with self.lock:
            self.bus.write_word_data(addr, reg, data)

class I2CDevice:
    def __init__(self, bus, addr):
        self.bus = bus
        self.addr = addr
        self.lock = threading.Lock()

    def read_register_byte(self, reg):
        with self.lock:
            return self.bus.read_byte_data(self.addr, reg)

    def read_register_word(self, reg):
        with self.lock:
            return self.bus.read_word_data(self.addr, reg)

    def read_register_multi(self, reg, count):
        with self.lock:
            return [self.bus.read_byte_data(self.addr, reg + i) for i in range(count)]

    def write_register_byte(self, reg, data):
        with self.lock:
            self.bus.write_byte_data(self.addr, reg, data)

    def write_register_word(self, reg, data):
        with self.lock:
            self.bus.write_word_data(self.addr, reg, data)

    def write_register_multi(self, reg, data):
        with self.lock:
            for i, value in enumerate(data):
                self.bus.write_byte_data(self.addr, reg + i, value)

class Odometry:
    def __init__(self, bus, addr):
        self.i2c = I2CDevice(bus, addr)
        self.robot_name = rospy.get_param('~robot_name', 'unknown_robot')
        self.odometry_pub = rospy.Publisher('/odometry', Odometry, queue_size=10)

    def read_pos(self):
        # Read position from the device
        position = self.i2c.read_register_word(0x00)
        # Publish position
        self.odometry_pub.publish(position)

    def can_read_pos(self):
        # You need to define this method to return True or False
       return True

def odometry_thread_fn(odometry):
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        if odometry.can_read_pos(): 
            if not odometry.read_pos():
                rospy.logerr("Error reading odometry")
        rate.sleep()

def main():
    rospy.init_node('i2c_manager', anonymous=True)
    rospy.loginfo("i2c_manager is starting!")

    bus1 = I2CBus(1)
    bus3 = I2CBus(3)

    odometry = Odometry(bus3, I2C_ADDR_ODOMETRY)

    odometry_thread = threading.Thread(target=odometry_thread_fn, args=(odometry,))
    odometry_thread.start()

    rospy.spin()

    odometry_thread.join()

if __name__ == '__main__':
    main()
