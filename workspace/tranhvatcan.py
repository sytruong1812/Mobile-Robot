import RPi.GPIO as GPIO
import time
from time import delay
dis = 30
IN1 = 13
IN2 = 12
IN3 = 21
IN4 = 20
ENA = 6
ENB = 26
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(IN1,GPIO.OUT)
GPIO.setup(IN2,GPIO.OUT)
GPIO.setup(IN3,GPIO.OUT)
GPIO.setup(IN4,GPIO.OUT)
GPIO.setup(ENA,GPIO.OUT)
GPIO.setup(ENB,GPIO.OUT)
PWMA = GPIO.PWM(ENA,500)
PWMB = GPIO.PWM(ENB,500)
PWMA.start(0)
PWMB.start(0)
def backward():
    GPIO.output(IN1,GPIO.HIGH)
    GPIO.output(IN2,GPIO.LOW)
    GPIO.output(IN3,GPIO.LOW)
    GPIO.output(IN4,GPIO.HIGH)
    PWMA.ChangeDutyCycle(100)
    PWMB.ChangeDutyCycle(100)
def stopmotor():
    GPIO.output(IN1,GPIO.LOW)
    GPIO.output(IN2,GPIO.LOW)
    GPIO.output(IN3,GPIO.LOW)
    GPIO.output(IN4,GPIO.LOW)
    PWMA.ChangeDutyCycle(0)
    PWMB.ChangeDutyCycle(0)
def forward():
    GPIO.output(IN1,GPIO.LOW)
    GPIO.output(IN2,GPIO.HIGH)
    GPIO.output(IN3,GPIO.HIGH)
    GPIO.output(IN4,GPIO.LOW)
    PWMA.ChangeDutyCycle(100)
    PWMB.ChangeDutyCycle(100)
def right():
    GPIO.output(IN1,GPIO.LOW)
    GPIO.output(IN2,GPIO.HIGH)
    GPIO.output(IN3,GPIO.LOW)
    GPIO.output(IN4,GPIO.LOW)
    PWMB.ChangeDutyCycle(100)
def left():
    GPIO.output(IN1,GPIO.LOW)
    GPIO.output(IN2,GPIO.LOW)
    GPIO.output(IN3,GPIO.HIGH)
    GPIO.output(IN4,GPIO.LOW)
    PWMA.ChangeDutyCycle(100)

while True :
    # Ultrasonic_sensor 1
    GPIO.setmode(GPIO.BCM)
    GPIO_TRIGGER_FORWARD = 23
    GPIO_ECHO_FORWARD = 24

    GPIO.setup(GPIO_TRIGGER_FORWARD,GPIO.OUT)
    GPIO.setup(GPIO_ECHO_FORWARD,GPIO.IN)
    GPIO.output(GPIO_TRIGGER_FORWARD, False)
    time.sleep(0.05)
    GPIO.output(GPIO_TRIGGER_FORWARD, True)
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER_FORWARD, False)
    start = time.time()
    while GPIO.input(GPIO_ECHO_FORWARD)==0:
        start = time.time()
    while GPIO.input(GPIO_ECHO_FORWARD)==1:
        stop = time.time()
    elapsed_forward = stop-start
    distance_forward = elapsed_forward * 17150
    print ("Distance_forward: ",distance_forward)
    time.sleep(0.002)

    # Ultrasonic_sensor 2
    GPIO.setmode(GPIO.BCM)
    GPIO_TRIGGER_RIGHT = 8
    GPIO_ECHO_RIGHT = 25

    GPIO.setup(GPIO_TRIGGER_RIGHT,GPIO.OUT)
    GPIO.setup(GPIO_ECHO_RIGHT,GPIO.IN)
    GPIO.output(GPIO_TRIGGER_RIGHT, False)
    time.sleep(0.05)
    GPIO.output(GPIO_TRIGGER_RIGHT, True)
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER_RIGHT, False)
    start = time.time()
    while GPIO.input(GPIO_ECHO_FORWARD)==0:
        start = time.time()
    while GPIO.input(GPIO_ECHO_RIGHT)==1:
        stop = time.time()
    elapsed_right = stop-start
    distance_right= elapsed_right * 17150
    print ("Distance_right:",distance_right)

    # Ultrasonic_sensor 3
    GPIO.setmode(GPIO.BCM)
    GPIO_TRIGGER_LEFT = 22
    GPIO_ECHO_LEFT = 27

    GPIO.setup(GPIO_TRIGGER_LEFT,GPIO.OUT)
    GPIO.setup(GPIO_ECHO_LEFT,GPIO.IN)
    GPIO.output(GPIO_TRIGGER_LEFT, False)
    time.sleep(0.05)
    GPIO.output(GPIO_TRIGGER_LEFT, True)
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER_LEFT, False)
    start = time.time()
    while GPIO.input(GPIO_ECHO_LEFT)==0:
        start = time.time()
    while GPIO.input(GPIO_ECHO_LEFT)==1:
        stop = time.time()
    elapsed_left = stop-start
    distance_left = elapsed_left * 17150
    print ("Distance_left: ",distance_left)
    time.sleep(0.002)
    
    if distance_left<dis and distance_forward>dis and distance_right>dis :
        right()
        time.sleep(0.2)
    elif distance_left>dis and distance_forward<dis and distance_right>dis :
        if distance_left>distance_right :
            left()
            time.sleep(0.2)
        if distance_left<distance_right :
            right()
            time.sleep(0.2)
        elif distance_left>dis and distance_forward>dis and distance_right<dis :
            left()
            time.sleep(0.2)
        elif distance_left<dis and distance_forward<dis and distance_right>dis :
            right()
            time.sleep(0.2)
        elif distance_left>dis and distance_forward<dis and distance_right<dis :
            left()
            time.sleep(0.2)
        elif distance_left<dis and distance_forward>dis and distance_right<dis :
            if distance_left>distance_right :
                left()
                time.sleep(0.2)
            if distance_left<distance_right :
                right()
                time.sleep(0.2)
            elif distance_left<dis and distance_forward<dis and distance_right<dis :
                stopmotor()
                time.sleep(1)
                backward()
                time.sleep(1)
                left()
                time.sleep(0.4)
            else :
                forward()