import time
import math
from rplidar import RPLidar


PORT_NAME = 'COM4'  

lidar = RPLidar(PORT_NAME,  baudrate=115200, timeout=3)
min_distance = 100000
angle_at_min_dist = 0

def print_data(angle, distance):
    print(f"distance: {distance}    angle: {angle}")

try:
    for scan in lidar.iter_scans():
        for (_, angle, distance) in scan:
            if distance > 0 and distance < min_distance:
                min_distance = distance
                angle_at_min_dist = angle
                if 0 < angle < 90 :
                    x = distance * math.cos(math.radians(angle))
                    y = distance * math.sin(math.radians(angle))
                    print("x={}, y={}".format(x, y))
                else:
                    continue
                
        if min_distance < 100000:
            print_data(angle_at_min_dist, min_distance)
            min_distance = 100000
            angle_at_min_dist = 0
        time.sleep(1)

except KeyboardInterrupt:
    lidar.stop_motor()
    lidar.disconnect()
