#!/usr/bin/python3

from rpi_lcd import LCD
import smbus
import time
import struct
import RPi.GPIO as GPIO

lcd = LCD()

I2C_ADDR_ODOMETRY = 0x22
I2C_LCD_ADDR = 0x27

# Define some device constants
LCD_WIDTH = 20  # Maximum characters per line

# Define some device constants
LCD_CHR = 1  # Mode - Sending data
LCD_CMD = 0  # Mode - Sending command

LCD_LINE_1 = 0x80  # LCD RAM address for the 1st line
LCD_LINE_2 = 0xC0  # LCD RAM address for the 2nd line
LCD_LINE_3 = 0x94

LCD_BACKLIGHT = 0x08  # On

ENABLE = 0b00000100  # Enable bit

# Timing constants
E_PULSE = 0.0005
E_DELAY = 0.0005

# Button GPIO pin
BUTTON_PIN = 17

# Open I2C interface
bus = smbus.SMBus(1)  # Rev 2 Pi uses 1

# Global variables for position and reset
total_position = 0
total_position_r = 0
reset_requested = False

def lcd_init():
    # Initialise display
    lcd_byte(0x33, LCD_CMD)  # 110011 Initialise
    lcd_byte(0x32, LCD_CMD)  # 110010 Initialise
    lcd_byte(0x06, LCD_CMD)  # 000110 Cursor move direction
    lcd_byte(0x0C, LCD_CMD)  # 001100 Display On,Cursor Off, Blink Off
    lcd_byte(0x28, LCD_CMD)  # 101000 Data length, number of lines, font size
    lcd_byte(0x01, LCD_CMD)  # 000001 Clear display
    time.sleep(E_DELAY)

def lcd_byte(bits, mode):
    # Send byte to data pins
    bits_high = mode | (bits & 0xF0) | LCD_BACKLIGHT
    bits_low = mode | ((bits << 4) & 0xF0) | LCD_BACKLIGHT

    # High bits
    bus.write_byte(I2C_LCD_ADDR, bits_high)
    lcd_toggle_enable(bits_high)

    # Low bits
    bus.write_byte(I2C_LCD_ADDR, bits_low)
    lcd_toggle_enable(bits_low)

def lcd_toggle_enable(bits):
    # Toggle enable
    time.sleep(E_DELAY)
    bus.write_byte(I2C_LCD_ADDR, (bits | ENABLE))
    time.sleep(E_PULSE)
    bus.write_byte(I2C_LCD_ADDR, (bits & ~ENABLE))
    time.sleep(E_DELAY)

def lcd_string(message, line):
    # Send string to display
    message = message.ljust(LCD_WIDTH, " ")
    lcd_byte(line, LCD_CMD)
    for i in range(LCD_WIDTH):
        lcd_byte(ord(message[i]), LCD_CHR)

def start_encoder():
    # Start counting
    bus.write_byte(I2C_ADDR_ODOMETRY, 0x01)

def stop_encoder():
    # Stop counting
    bus.write_byte(I2C_ADDR_ODOMETRY, 0x10)

def read_distance():
    global total_position, total_position_r, reset_requested
    wheel_diameter = 6.51  # in cm
    wheel_circumference = wheel_diameter * 3.14159  # in cm
    while True:
        start_encoder() # Start counting
        time.sleep(0.001) # Wait for the encoder to count

        stop_encoder()  # Stop counting

        if reset_requested:
            total_position = 0
            total_position_r = 0
            distance_moy = 0
            lcd_string("Dist l: 0.0 cm", LCD_LINE_1)
            lcd_string("Dist r: 0.0 cm", LCD_LINE_2)
            lcd_string("Dist_tot: 0.0 cm", LCD_LINE_3)
            reset_requested = False
            continue

        # Read the position encoder left
        bus.write_byte(I2C_ADDR_ODOMETRY, 0x31)
        # Read the four bytes
        byte1 = bus.read_byte(I2C_ADDR_ODOMETRY)
        byte2 = bus.read_byte(I2C_ADDR_ODOMETRY)
        byte3 = bus.read_byte(I2C_ADDR_ODOMETRY)
        byte4 = bus.read_byte(I2C_ADDR_ODOMETRY)

        position_bytes = struct.unpack('>I', bytearray([byte4, byte3, byte2, byte1]))[0]

        bus.write_byte(I2C_ADDR_ODOMETRY, 0x32)
        # Read the four bytes
        byte1_r = bus.read_byte(I2C_ADDR_ODOMETRY)
        byte2_r = bus.read_byte(I2C_ADDR_ODOMETRY)
        byte3_r = bus.read_byte(I2C_ADDR_ODOMETRY)
        byte4_r = bus.read_byte(I2C_ADDR_ODOMETRY)

        position_bytes_r = struct.unpack('>I', bytearray([byte4_r, byte3_r, byte2_r, byte1_r]))[0]

        if byte2 < 150:
            total_position += position_bytes
            distance_cm = ((total_position * wheel_circumference) / 4096 ) * 14.79

            print("Déplacement_l:", distance_cm)
            print("position_bytes: " , position_bytes)
            print("byte1: ", byte1)
            print("byte2: ", byte2)
            print("byte3: ", byte3)
            print("byte4: ", byte4)
            message = "Dist l: {:.1f} cm".format(distance_cm)
            lcd_string(message, LCD_LINE_1)
            #time.sleep(0.1)

        if byte2_r < 150:
            total_position_r += position_bytes_r
            distance_cm_r = ((total_position_r * wheel_circumference) / 4096 ) * 8.682
            print("Déplacement_r:", distance_cm_r)
            print("position_bytes_r: " , position_bytes_r)
            print("byte1: ", byte1_r)
            print("byte2: ", byte2_r)
            print("byte3: ", byte3_r)
            print("byte4: ", byte4_r)
            message_r = "Dist r: {:.1f} cm".format(distance_cm_r)
            lcd_string(message_r, LCD_LINE_2)
            #time.sleep(0.1)
    
        distance_moy = (distance_cm_r + distance_cm) / 2
        print("Moy:", distance_moy)
        message_moy = "Dist_tot: {:.1f} cm".format(distance_moy)
        lcd_string(message_moy, LCD_LINE_3)

def reset_distance(channel):
    global reset_requested
    reset_requested = True

try:
    # Set up the GPIO for the button
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.add_event_detect(BUTTON_PIN, GPIO.RISING, callback=reset_distance, bouncetime=300)

    lcd_init()
    read_distance()

except KeyboardInterrupt:
    stop_encoder()  # Stop the encoder before exit
    pass

finally:
    lcd_byte(0x01, LCD_CMD)  # Clear display
    bus.close()
    GPIO.cleanup()  # Clean up GPIO
    
    
    
"""
#!/usr/bin/python3

from rpi_lcd import LCD
import smbus
import time
import struct
-----------------------------------------------------------------
lcd = LCD()

I2C_ADDR_ODOMETRY = 0x22
I2C_LCD_ADDR = 0x27

# Define some device constants
LCD_WIDTH = 16  # Maximum characters per line

# Define some device constants
LCD_CHR = 1  # Mode - Sending data
LCD_CMD = 0  # Mode - Sending command

LCD_LINE_1 = 0x80  # LCD RAM address for the 1st line
LCD_LINE_2 = 0xC0  # LCD RAM address for the 2nd line

LCD_BACKLIGHT = 0x08  # On

ENABLE = 0b00000100  # Enable bit

# Timing constants
E_PULSE = 0.0005
E_DELAY = 0.0005

# Open I2C interface
bus = smbus.SMBus(1)  # Rev 2 Pi uses 1

def lcd_init():
    # Initialise display
    lcd_byte(0x33, LCD_CMD)  # 110011 Initialise
    lcd_byte(0x32, LCD_CMD)  # 110010 Initialise
    lcd_byte(0x06, LCD_CMD)  # 000110 Cursor move direction
    lcd_byte(0x0C, LCD_CMD)  # 001100 Display On,Cursor Off, Blink Off
    lcd_byte(0x28, LCD_CMD)  # 101000 Data length, number of lines, font size
    lcd_byte(0x01, LCD_CMD)  # 000001 Clear display
    time.sleep(E_DELAY)


def lcd_byte(bits, mode):
    # Send byte to data pins
    bits_high = mode | (bits & 0xF0) | LCD_BACKLIGHT
    bits_low = mode | ((bits << 4) & 0xF0) | LCD_BACKLIGHT

    # High bits
    bus.write_byte(I2C_LCD_ADDR, bits_high)
    lcd_toggle_enable(bits_high)

    # Low bits
    bus.write_byte(I2C_LCD_ADDR, bits_low)
    lcd_toggle_enable(bits_low)


def lcd_toggle_enable(bits):
    # Toggle enable
    time.sleep(E_DELAY)
    bus.write_byte(I2C_LCD_ADDR, (bits | ENABLE))
    time.sleep(E_PULSE)
    bus.write_byte(I2C_LCD_ADDR, (bits & ~ENABLE))
    time.sleep(E_DELAY)


def lcd_string(message, line):
    # Send string to display
    message = message.ljust(LCD_WIDTH, " ")
    lcd_byte(line, LCD_CMD)
    for i in range(LCD_WIDTH):
        lcd_byte(ord(message[i]), LCD_CHR)

def start_encoder():
    # Start counting
    bus.write_byte(I2C_ADDR_ODOMETRY, 0x01)

def stop_encoder():
    # Stop counting
    bus.write_byte(I2C_ADDR_ODOMETRY, 0x10)

def read_distance():
    total_position = 0
    total_position_r = 0
    wheel_diameter = 6.51  # in cm
    wheel_circumference = wheel_diameter * 3.14159  # in cm
    while True:
        start_encoder() # Start counting
        time.sleep(0.001) # Wait for the encoder to count
        
        stop_encoder()  # Stop counting
        # Read the position encoder left
        bus.write_byte(I2C_ADDR_ODOMETRY, 0x31)
        # Read the four bytes
        byte1 = bus.read_byte(I2C_ADDR_ODOMETRY)
        byte2 = bus.read_byte(I2C_ADDR_ODOMETRY)
        byte3 = bus.read_byte(I2C_ADDR_ODOMETRY)
        byte4 = bus.read_byte(I2C_ADDR_ODOMETRY)
        
        
        
        position_bytes = struct.unpack('>I', bytearray([byte4, byte3, byte2, byte1]))[0]
        
        bus.write_byte(I2C_ADDR_ODOMETRY, 0x32)
        # Read the four bytes
        byte1_r = bus.read_byte(I2C_ADDR_ODOMETRY)
        byte2_r = bus.read_byte(I2C_ADDR_ODOMETRY)
        byte3_r = bus.read_byte(I2C_ADDR_ODOMETRY)
        byte4_r = bus.read_byte(I2C_ADDR_ODOMETRY)
        
        
        
        position_bytes = struct.unpack('>I', bytearray([byte4, byte3, byte2, byte1]))[0]
        
        position_bytes_r = struct.unpack('>I', bytearray([byte4_r, byte3_r, byte2_r, byte1_r]))[0]

        if byte2 < 150:
        	

        
        	total_position += position_bytes

        	distance_cm = ((total_position * wheel_circumference) / 4096 ) * 14.79 # Le fois 100 pour l'affichage

        	print("Déplacement_l:", distance_cm)
        	print("position_bytes: " , position_bytes)
        	print("byte1: ", byte1)
        	print("byte2: ", byte2)
        	print("byte3: ", byte3)
        	print("byte4: ", byte4)
        	message = "Dist l: {:.1f} cm".format(distance_cm)
        	lcd_string(message, LCD_LINE_1)
        	#time.sleep(0.1)
        
        if byte2_r < 150:
        	total_position_r += position_bytes_r
        	distance_cm_r = ((total_position_r * wheel_circumference) / 4096 ) * 8.682
        	print("Déplacement_r:", distance_cm_r)
        	print("position_bytes_r: " , position_bytes_r)
        	print("byte1: ", byte1_r)
        	print("byte2: ", byte2_r)
        	print("byte3: ", byte3_r)
        	print("byte4: ", byte4_r)
        	message_r = "Dist r: {:.1f} cm".format(distance_cm_r)
        	lcd_string(message_r, LCD_LINE_2)
        	#time.sleep(0.1)

try:
    lcd_init()
    read_distance()

except KeyboardInterrupt:
    stop_encoder()  # Stop the encoder before exit
    pass

finally:
    lcd_byte(0x01, LCD_CMD)  # Clear display
    bus.close()
    
"""

