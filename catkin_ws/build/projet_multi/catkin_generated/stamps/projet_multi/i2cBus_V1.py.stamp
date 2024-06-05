#!/usr/bin/python3

"""
    Program: LCD1602 Demo (lcd-hello.py)
    Author:  M. Heidenreich, (c) 2020

    Description:
    
    This code is provided in support of the following YouTube tutorial:
    https://youtu.be/DHbLBTRpTWM

    This example shows how to use the LCD1602 I2C display with Raspberry Pi.

    THIS SOFTWARE AND LINKED VIDEO TOTORIAL ARE PROVIDED "AS IS" AND THE
    AUTHOR DISCLAIMS ALL WARRANTIES INCLUDING ALL IMPLIED WARRANTIES OF
    MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
    ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
    WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
    ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
    OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
"""

from signal import signal, SIGTERM, SIGHUP, pause
from rpi_lcd import LCD
import smbus
import time

# Define I2C address of the slave
I2C_ADDR_ODOMETRY = 0x22

# Initialize the LCD
lcd = LCD()

# Initialize the I2C bus
bus = smbus.SMBus(1)

# Function to read distance from the I2C slave
def read():
    recieve = bus.read_word_data(I2C_ADDR_ODOMETRY, 0x31)  # Read distance data from the slave
    return recieve

# Function to enter test mode
def enter_test_mode():
    lcd.clear()
    lcd.text("Test Mode", 1)
    counter = 0
    while True:
        lcd.text(f"Counter: {counter}", 2)
        time.sleep(0.01)
        counter += 1

# Function to exit test mode
def exit_test_mode():
    lcd.clear()
    lcd.text("Exiting Test", 1)
    time.sleep(0.01)

# Function to safely exit the program
def safe_exit(signum, frame):
    exit(1)

try:
    signal(SIGTERM, safe_exit)
    signal(SIGHUP, safe_exit)

    test_mode = False

    while True:
        # Check if 't' is pressed to toggle test mode
        if input() == 't':
            test_mode = not test_mode

            # Enter or exit test mode based on current state
            if test_mode:
                enter_test_mode()
            else:
                exit_test_mode()

        # Read the distance from the I2C slave
        recieve = read()

        # Display distance or test mode message based on the current mode
        if not test_mode:
            lcd.clear()
            lcd.text(f"Distance: {recieve/100.0:.2f} m", 1)
        else:
            lcd.clear()
            lcd.text(f"Test Mode", 1)

            # Print the distance read from the I2C slave
            print(f"Distance: {recieve/100.0:.2f} m")

        time.sleep(0.1)

except KeyboardInterrupt:
    pass

finally:
    lcd.clear()

