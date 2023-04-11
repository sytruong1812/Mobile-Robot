import time
import math
from firebase import firebase
import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.GPIO as GPIO
from Adafruit_BBIO.Encoder import RotaryEncoder, eQEP0, eQEP2
import pid_roirac_vitri_phai

"""---------------------Firebase------------------------"""
firebase = firebase.FirebaseApplication('https://mobile-robot-6269b-default-rtdb.firebaseio.com/', None)

"""---------------------PWM-----------------------------"""
RPWM_A = "P8_13"
LPWM_A = "P8_19"
RPWM_B = "P9_16"
LPWM_B = "P9_14"
# PWM duty cycle. It must have a value from 0 to 100.
PWM.start(RPWM_A, 100, 2000, 1)
PWM.start(LPWM_A, 100, 2000, 1)
PWM.start(RPWM_B, 100, 2000, 1)
PWM.start(LPWM_B, 100, 2000, 1)
"""-------------------ENCODER----------------------------"""
myEncoderA = RotaryEncoder(eQEP2)       #eQEP0    P9.27    P9.42
myEncoderB = RotaryEncoder(eQEP0)       #eQEP2    P8.11    P8.12
# Chế độ tuyệt đối: vị trí bắt đầu từ 0 và được tăng hoặc giảm theo chuyển động của bộ mã hóa
myEncoderA.setAbsolute()
myEncoderB.setAbsolute()
wheel_A = 0
wheel_B = 0
"""----------------------Ultrasonic Sensor----------------------"""
GPIO.setup("P9_15",GPIO.OUT) #Trigger
GPIO.setup("P9_12",GPIO.IN)  #Echo
#Security
GPIO.output("P9_15", False)
# time.sleep(0.5)
"""--------------------------KINETICS----------------------------"""
D = 9,5      # Đường kính bánh xe D = 95mm -> R = D/2
d = 280     # Khoảng cách giữa 2 bánh xe
phi_right = 0   # Góc quay của bánh phải
phi_left = 0    # Góc quay của bánh trái
w_right = 0     # Vận tốc góc quay bánh phải
w_left = 0      # Vận tốc góc quay bánh trái
v = 0           # Vận tốc
theta = 0       # Góc đinh hướng của xe
x = 0           
y = 0

def Forward(pwm):
    PWM.set_duty_cycle(RPWM_A, pwm)
    PWM.set_duty_cycle(LPWM_A, 0)
    PWM.set_duty_cycle(RPWM_B, pwm)
    PWM.set_duty_cycle(LPWM_B, 0)
def Backward(pwm):
    PWM.set_duty_cycle(RPWM_A, 0)
    PWM.set_duty_cycle(LPWM_A, pwm)
    PWM.set_duty_cycle(RPWM_B, 0)
    PWM.set_duty_cycle(LPWM_B, pwm)
def Right_Thuan(pwm):
    PWM.set_duty_cycle(RPWM_A, pwm)
    PWM.set_duty_cycle(LPWM_A, 0)
    PWM.set_duty_cycle(RPWM_B, 0)
    PWM.set_duty_cycle(LPWM_B, 0)
def Right_Nguoc(pwm):
    PWM.set_duty_cycle(RPWM_A, 0)
    PWM.set_duty_cycle(LPWM_A, pwm)
    PWM.set_duty_cycle(RPWM_B, 0)
    PWM.set_duty_cycle(LPWM_B, 0)
def Left_Thuan(pwm):
    PWM.set_duty_cycle(RPWM_A, 0)
    PWM.set_duty_cycle(LPWM_A, 0)
    PWM.set_duty_cycle(RPWM_B, pwm)
    PWM.set_duty_cycle(LPWM_B, 0)
def Left_Nguoc(pwm):
    PWM.set_duty_cycle(RPWM_A, 0)
    PWM.set_duty_cycle(LPWM_A, 0)
    PWM.set_duty_cycle(RPWM_B, 0)
    PWM.set_duty_cycle(LPWM_B, pwm)
def Stopmotor(pwm):
    PWM.set_duty_cycle(RPWM_A, pwm)
    PWM.set_duty_cycle(LPWM_A, pwm)
    PWM.set_duty_cycle(RPWM_B, pwm)
    PWM.set_duty_cycle(LPWM_B, pwm)
    
# Xe quay 1 góc 90 độ

# Xe quay ngược 1 góc 90 độ

# Xe quay 1 góc 180 độ

# Xe quay ngược 1 góc 180 độ

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
    distance = pulseDuration * 17150        # multiply with the sonic speed (34250 cm/s)
    distance = round(distance, 2)   # Làm tròn
    return distance

"""
    Đường kính bánh xe: Dn = 95mm = 9.5cm
    pi = 3.14159
    Chu vi bánh xe: Dn*pi=9.5*3.14=29.83 (cm)
    Độ phân giải encoder 2400 xung/vòng
    Số mm/1xung=29.83/2400=0.01242916666 (cm)
"""

def controlFirebase():
    result = firebase.get('/Control', None)
    if result["Forward"] == 1:
        Forward(45)
        print("Forward")
    elif result["Backward"] == 1:
        Backward(45)
        print("Backward")
    elif result["Left_Thuan"] == 1:
        Left_Thuan(45)
        print("Left_Thuận")
    elif result["Left_Nguoc"] == 1:
        Left_Nguoc(45)
        print("Left_Ngược")
    elif result["Right_Thuan"] == 1:
        Right_Thuan(45)
        print("Right_Thuận")
    elif result["Right_Nguoc"] == 1:
        Right_Nguoc(45)
        print("Right_Ngược")
    else:
        Stopmotor(45)
        print("Stopmotor")

def kinetics():
    v = float(R * (w_right + w_left)) / 2
    theta = float(R * (w_right - w_left)) / d
    x = v * math.cos(theta)
    y = v * math.sin(theta)

def distanceForward(s):    #input parameter là quãng đường muốn tới (cm)
    result = firebase.get('/Control', None)
    if result["Forward"] == 1:
        Forward(45)
        # Nhận vị trí hiện tại
        positionA = myEncoderA.position
        positionB = myEncoderB.position
        N = (int)(s / (math.pi * D))
        if positionA == N & positionB == N:
            Forward(0)
        print("Forward")
try: 
    while True:
        # controlFirebase()
        distanceForward(100)
except KeyboardInterrupt:
    exit()


        




