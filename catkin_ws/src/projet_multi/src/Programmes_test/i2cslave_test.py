
import smbus
import time

I2C_ADDR_ODOMETRY = 0x22

# Timing constants
E_DELAY = 0.0005

# Open I2C interface
bus = smbus.SMBus(1)  # Rev 2 Pi uses 1

def read_distance():
    while True:
        recieve = bus.read_word_data(I2C_ADDR_ODOMETRY, 0x31)
        message = "Slave send: {:.2f} m".format(recieve)  # Conversion en mètres
        print(message)
        time.sleep(0.001)


try:
    read_distance()

except KeyboardInterrupt:
    pass

finally:
    bus.close()

