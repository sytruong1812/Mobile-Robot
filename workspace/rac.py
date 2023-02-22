# beaglebone_black
import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.PWM as PWM
from pyrebase import pyrebase
from pylidar import lidar
import math
import time 
import numpy as np



# Firebase configuration
config = {
    "apiKey": "AIzaSyDeZ6Zp276tWMonX6f_XuN6X7muoFSph9g",
    "authDomain": "moblie-robot.firebaseapp.com",
    "databaseURL": "https://moblie-robot-default-rtdb.firebaseio.com",
    "storageBucket": "moblie-robot.appspot.com",
}
firebase = pyrebase.initialize_app(config)
db = firebase.database()
cred = credentials.Certificate("path/to/serviceAccountKey.json")
firebase_admin.initialize_app(cred, {
    'databaseURL': 'https://moblie-robot-default-rtdb.firebaseio.com/'
})
ref = db.reference('robot')

# Khởi tạo các biến
distance = 0
current_position = 0
previous_position = 0
current_time = time.time()
previous_time = current_time
distance_traveled = 0

#Điều chỉnh tốc độ robot
def update_speed_speed(change):
    new_speed = change.data
    PWM.set_duty_cycle("P9_14", float(new_speed))
speed_ref = db.reference('robot/speed')
speed_ref.listen(update_speed_speed)

# Motor pins
IN1 = "P8_10"
IN2 = "P8_11"
IN3 = "P8_12"
IN4 = "P8_13"

# Encoder
GPIO.setup("P8_7", GPIO.IN)
GPIO.setup("P8_9", GPIO.IN)

# Lidar
PORT_NAME = '/dev/ttyUSB0'
lidar = rplidar.RPLidar(PORT_NAME)
ser = serial.Serial('/dev/ttyACM0', 9600, timeout=0.1)
ROBOT_WIDTH = 10.0  # cm
ROBOT_HEIGHT = 10.0  # cm
ROBOT_RADIUS = math.sqrt((ROBOT_WIDTH / 2) ** 2 + (ROBOT_HEIGHT / 2) ** 2)  # cm

# Hàm đọc giá trị Encoder
def read_encoder(channel):
    global current_position
    current_position = current_position + 1

# Gán callback cho GPIO
GPIO.add_event_detect("P8_7", GPIO.BOTH, callback=read_encoder)
GPIO.add_event_detect("P8_9", GPIO.BOTH, callback=read_encoder)

#Hàm đọc giá trị từ Lidar
def polar_to_cartesian(r, theta):
    x = r * math.cos(math.radians(theta))
    y = r * math.sin(math.radians(theta))
    return x, y

def process_lidar_data(data):
    angles = np.array([d[1] for d in data])
    distances = np.array([d[2] for d in data])
    x, y = polar_to_cartesian(distances, angles)
    return x, y

def get_lidar_data():
    data = []
    for scan in lidar.iter_scans():
        data.extend(scan)
        break
    return data

def get_encoder_data():
    data = ser.readline().decode('utf-8')
    return data

def get_robot_position():
    lidar_data = get_lidar_data()
    x, y = process_lidar_data(lidar_data)
    encoder_data = get_encoder_data()
    return x, y, encoder_data

def update_robot_position():
    x, y, encoder_data = get_robot_position()
    ref.update
    (
        {
        'position': {
            'x': x,
            'y': y,
            'encoder': encoder_data
        }
        }
    )
    
            
    
# Motor control functions
def moveForward(speed):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    PWM.start(IN1, speed)

def moveBackward(speed):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    PWM.start(IN1, speed)

def turnRight(speed):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    PWM.start(IN1, speed)

def turnLeft(speed):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    PWM.start(IN1, speed)

def stop():
    PWM.stop(IN1)
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
     
# Firebase event handlers
def on_direction_changed(event):
    direction = event["data"]
    if direction == "forward":
        moveForward(100)
    elif direction == "backward":
        moveBackward(100)
    elif direction == "left":
        turnLeft(100)
    elif direction == "right":
        turnRight(100)
    else:
        stop()

# Firebase setup
direction_ref = db.child("direction")
direction_ref.stream(on_direction_changed)

# Main loop
while True:
# Đọc giá trị từ encoder
    current_time = time.time()
    time_difference = current_time - previous_time
    previous_time = current_time
    distance_traveled = (current_position - previous_position) * (0.15 * 3.14) / 32
    previous_position = current_position
    
# Đọc giá trị từ Lidar
    x, y, encoder_data = get_robot_position()
    ref.update({
        'position': {
            'x': x,
            'y': y,
            'encoder': encoder_data
        }
    })
    time.sleep(0.1)

# Đọc giá trị từ Firebase
    speed_ref = db.reference('robot/speed')
    speed_ref.listen(update_speed_speed)
    direction_ref = db.child("direction")
    direction_ref.stream(on_direction_changed)
    time.sleep(0.1)