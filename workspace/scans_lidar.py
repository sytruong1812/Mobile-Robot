import time
import math
from rplidar import RPLidar
# import Adafruit_BBIO.GPIO as GPIO

# GPIO.setup("P9_12", GPIO.OUT)
# GPIO.output("P9_12", GPIO.HIGH)

"""
config-pin P9_26 uart
config-pin P9_24 uart
stty -F /dev/ttyO1 115200 cs8 -cstopb -parity
"""

PORT_NAME = 'COM4'
#PORT_NAME = '/dev/ttyO1'    # UART1: /dev/ttyO1, Rx: P9_26, Tx: P9_24

lidar = RPLidar(PORT_NAME,  baudrate = 115200, timeout = 1)

# def print_data(angle, distance):
#     print(f"distance: {distance / 10}cm      angle: {angle}")

try:
    for scan in lidar.iter_scans():
        for (_, angle, distance) in scan:
            if  angle < 0:
                print(f"distance: {distance / 10}cm      angle: {angle}")
                x = (distance / 10) * math.cos(math.radians(angle))
                y = (distance / 10) * math.sin(math.radians(angle))
                print("x={}cm, y={}cm".format(x, y))
            else:
                continue
except KeyboardInterrupt:
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()
    # GPIO.output("P9_12", GPIO.LOW)
