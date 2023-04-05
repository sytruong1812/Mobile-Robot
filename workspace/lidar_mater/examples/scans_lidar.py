import time
import math
from rplidar import RPLidar

PORT_NAME = 'COM4'  
lidar = RPLidar(PORT_NAME,  baudrate = 115200, timeout = 3)

def print_data(angle, distance):
    print(f"distance: {distance}    angle: {angle}")

try:
    for scan in lidar.iter_scans():
        for (_, angle, distance) in scan:
            if 0 < angle < 90 :
                print_data(angle, distance)
                x = distance * math.cos(math.radians(angle))
                y = distance * math.sin(math.radians(angle))
                print("x={}, y={}".format(x, y))
            else:
                continue
except KeyboardInterrupt:
    lidar.stop_motor()
    lidar.disconnect()
