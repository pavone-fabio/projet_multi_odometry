import qwiic_vl53l1x
import time
import sys
import qwiic_oled_display

myOLED = qwiic_oled_display.QwiicOledDisplay()


def display(data):
    if myOLED.is_connected() == False:
        print("The Qwiic OLED Display isn't connected to the system. Please check your connection", file=sys.stderr)
        return

    myOLED.begin()
    myOLED.clear(myOLED.PAGE)
    myOLED.print(data)
    myOLED.display()

def runExample():
    print("\nSparkFun VL53L1X Example 1\n")
    mySensor = qwiic_vl53l1x.QwiicVL53L1X()

    if mySensor._begin() == False:
        print("The Qwiic VL53L1X device isn't connected to the system. Please check your connection", file=sys.stderr)
        return

    mySensor.sensor_init()
    last_time = 0

    while True:
        try:
            current_time = time.time()
            mySensor.start_ranging()             # Write configuration bytes to initiate measurement
            time.sleep(0.005)
            distance = mySensor.get_distance()   # Get the result of the measurement from the sensor
            time.sleep(0.005)
            mySensor.stop_ranging()

            print("Distance(mm): %s" % distance)
            if current_time - last_time >=1:
                display("Dist :" + str(distance))
                last_time = current_time
        except Exception as e:
            print(e)

runExample()


# import qwiic_vl53l1x
# import time
# import sys
# import qwiic_oled_display

# myOLED = qwiic_oled_display.QwiicOledDisplay()

# def display(data):
#     if myOLED.is_connected() == False:
#         print("The Qwiic OLED Display isn't connected to the system. Please check your connection", file=sys.stderr)
#         return

#     myOLED.begin()
#     myOLED.clear(myOLED.PAGE)
#     myOLED.print(data)
#     myOLED.display()

# def runExample():
#     print("\nSparkFun VL53L1X Example 1\n")
#     mySensor = qwiic_vl53l1x.QwiicVL53L1X()

#     if mySensor._begin() == False:
#         print("The Qwiic VL53L1X device isn't connected to the system. Please check your connection", file=sys.stderr)
#         return

#     mySensor.sensor_init()

#     while True:
#         try:
#             mySensor.start_ranging()             # Write configuration bytes to initiate measurement
#             time.sleep(0.005)
#             distance = mySensor.get_distance()   # Get the result of the measurement from the sensor
#             time.sleep(0.005)
#             mySensor.stop_ranging()

#             print("Distance(mm): %s" % distance)
#             display("Dist :" + str(distance))
            
#         except Exception as e:
#             print(e)

# runExample()
