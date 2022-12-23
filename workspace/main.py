import os
import time
import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.GPIO as GPIO
from Adafruit_BBIO.Encoder import RotaryEncoder, eQEP0, eQEP2

"""MotorA"""
IN1 = "P8_7"
IN2 = "P8_8"

"""MotorB"""
IN3 = "P8_9"
IN4 = "P8_10"

GPIO.setup(IN1,GPIO.OUT)
GPIO.setup(IN2,GPIO.OUT)
GPIO.setup(IN3,GPIO.OUT)
GPIO.setup(IN4,GPIO.OUT)

"""PWM"""
ENA = "P9_14"
ENB = "P9_16"

#PWM duty cycle. It must have a value from 0 to 100.
PWM.start(ENA, 0, 2000, 0)
PWM.start(ENB, 0, 2000, 0)

# Khởi tạo lớp để truy cập kênh eQEP1, eQEP2 và chỉ khởi tạo kênh đó
myEncoderA = RotaryEncoder(eQEP0)
myEncoderB = RotaryEncoder(eQEP2)
# Chế độ tuyệt đối: vị trí bắt đầu từ 0 và được tăng hoặc giảm theo chuyển động của bộ mã hóa
myEncoderA.setAbsolute()
myEncoderB.setAbsolute()

"""Ultrasonic Sensor"""
GPIO.setup("P9_15",GPIO.OUT) #Trigger
GPIO.setup("P9_12",GPIO.IN)  #Echo

#Security
GPIO.output("P9_15", False)
time.sleep(0.5)


def backward():
    GPIO.output(IN1,GPIO.HIGH)
    GPIO.output(IN2,GPIO.LOW)
    GPIO.output(IN3,GPIO.LOW)
    GPIO.output(IN4,GPIO.HIGH)
    PWM.set_duty_cycle(ENA, 50)
    PWM.set_duty_cycle(ENB, 50)
def stopmotor():
    GPIO.output(IN1,GPIO.LOW)
    GPIO.output(IN2,GPIO.LOW)
    GPIO.output(IN3,GPIO.LOW)
    GPIO.output(IN4,GPIO.LOW)
    PWM.set_duty_cycle(ENA, 0)
    PWM.set_duty_cycle(ENB, 0)
def forward():
    GPIO.output(IN1,GPIO.LOW)
    GPIO.output(IN2,GPIO.HIGH)
    GPIO.output(IN3,GPIO.HIGH)
    GPIO.output(IN4,GPIO.LOW)
    PWM.set_duty_cycle(ENA, 50)
    PWM.set_duty_cycle(ENB, 50)
def right():
    GPIO.output(IN1,GPIO.LOW)
    GPIO.output(IN2,GPIO.HIGH)
    GPIO.output(IN3,GPIO.LOW)
    GPIO.output(IN4,GPIO.LOW)
    PWM.set_duty_cycle(ENB, 50)
def left():
    GPIO.output(IN1,GPIO.LOW)
    GPIO.output(IN2,GPIO.LOW)
    GPIO.output(IN3,GPIO.HIGH)
    GPIO.output(IN4,GPIO.LOW)
    PWM.set_duty_cycle(ENA, 50)
    
# Xe quay 1 góc 90 độ

# Xe quay ngược 1 góc 90 độ

# Xe quay 1 góc 180 độ

# Xe quay ngược 1 góc 180 độ


# Đường kính bánh xe: Dn = 95mm
# pi = 3.14
# Chu vi bánh xe: Dn*pi=95*3.14=298.3 (mm)
# Độ phân giải encoder 2400 xung/vòng
# Số mm/1xung=298.3/2400=0,1242916666666667 (mm)

def Encoder_Wheel():
    # Nhận vị trí hiện tại
    positionA = myEncoderA.position
    positionB = myEncoderB.position
    # Pulse -> Distance
    wheel_A = positionA*(198.3/2400)
    print ("Distance_Wheel_A: ",wheel_A,"mm")
    wheel_B = positionB*(298.3/2400)
    print ("Distance_Wheel_B: ",wheel_B,"mm")

def distanceMeasurement(TRIG,ECHO):
    GPIO.output(TRIG, True)     # set Trigger to HIGH
    time.sleep(0.00001)         # set Trigger after 0.01ms to LOW
    GPIO.output(TRIG, False)

    pulseStart = 0
    pulseEnd = 0

    while GPIO.input(ECHO) == 0:
        pulseStart = time.time()
    while GPIO.input(ECHO) == 1:
        pulseEnd = time.time()

    pulseDuration = pulseEnd - pulseStart
    distance = pulseDuration * 17150        # multiply with the sonic speed (34300 cm/s)
    distance = round(distance, 2)   # Làm tròn số 
    return distance

while True:
    control = input("Control DC Motor: ")
    if control == "s":
        stopmotor()
        Encoder_Wheel()
    if control == "w":
        forward()
        Encoder_Wheel()
    if control == "x":
        backward()
        Encoder_Wheel()
    if control == "a":
        left()
        Encoder_Wheel()
    if control == "d":
        right()
        Encoder_Wheel()
    if control == "e":
        PWM.stop(ENA)
        PWM.stop(ENB)
        PWM.cleanup()
        myEncoderA.zero()   # Đặt lại vị trí encoderA về 0
        myEncoderB.zero()   # Đặt lại vị trí encoderB về 0


    