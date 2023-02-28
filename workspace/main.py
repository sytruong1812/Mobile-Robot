import time
import math
import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.GPIO as GPIO
from Adafruit_BBIO.Encoder import RotaryEncoder, eQEP0, eQEP2


"""PWM"""
RPWM_A = "P9_16"
LPWM_A = "P9_14"

RPWM_B = "P8_13"
LPWM_B = "P8_19"

#PWM duty cycle. It must have a value from 0 to 100.
PWM.start(RPWM_A, 100, 2000, 1)
PWM.start(LPWM_A, 100, 2000, 1)

PWM.start(RPWM_B, 100, 2000, 1)
PWM.start(LPWM_B, 100, 2000, 1)

# Khởi tạo lớp để truy cập kênh eQEP1, eQEP2 và chỉ khởi tạo kênh đó
myEncoderA = RotaryEncoder(eQEP2)       #eQEP0    P9.27    P9.42
myEncoderB = RotaryEncoder(eQEP0)       #eQEP2    P8.11    P8.12
# Chế độ tuyệt đối: vị trí bắt đầu từ 0 và được tăng hoặc giảm theo chuyển động của bộ mã hóa
myEncoderA.setAbsolute()
myEncoderB.setAbsolute()

wheel_A = 0
wheel_B = 0

"""Ultrasonic Sensor"""
GPIO.setup("P9_15",GPIO.OUT) #Trigger
GPIO.setup("P9_12",GPIO.IN)  #Echo
#Security
GPIO.output("P9_15", False)
time.sleep(0.5)

def Forward():
    PWM.set_duty_cycle(RPWM_A, 45)
    PWM.set_duty_cycle(LPWM_A, 0)
    PWM.set_duty_cycle(RPWM_B, 45)
    PWM.set_duty_cycle(LPWM_B, 0)
def Backward():
    PWM.set_duty_cycle(RPWM_A, 0)
    PWM.set_duty_cycle(LPWM_A, 45)
    PWM.set_duty_cycle(RPWM_B, 0)
    PWM.set_duty_cycle(LPWM_B, 45)
def Right_Thuan():
    PWM.set_duty_cycle(RPWM_A, 45)
    PWM.set_duty_cycle(LPWM_A, 0)
    PWM.set_duty_cycle(RPWM_B, 0)
    PWM.set_duty_cycle(LPWM_B, 0)
def Right_Nguoc():
    PWM.set_duty_cycle(RPWM_A, 0)
    PWM.set_duty_cycle(LPWM_A, 45)
    PWM.set_duty_cycle(RPWM_B, 0)
    PWM.set_duty_cycle(LPWM_B, 0)
def Left_Thuan():
    PWM.set_duty_cycle(RPWM_A, 0)
    PWM.set_duty_cycle(LPWM_A, 0)
    PWM.set_duty_cycle(RPWM_B, 45)
    PWM.set_duty_cycle(LPWM_B, 0)
def Left_Nguoc():
    PWM.set_duty_cycle(RPWM_A, 0)
    PWM.set_duty_cycle(LPWM_A, 0)
    PWM.set_duty_cycle(RPWM_B, 0)
    PWM.set_duty_cycle(LPWM_B, 45)
def Stopmotor():
    PWM.set_duty_cycle(RPWM_A, 0)
    PWM.set_duty_cycle(LPWM_A, 0)
    PWM.set_duty_cycle(RPWM_B, 0)
    PWM.set_duty_cycle(LPWM_B, 0)
    
# Xe quay 1 góc 90 độ

# Xe quay ngược 1 góc 90 độ

# Xe quay 1 góc 180 độ

# Xe quay ngược 1 góc 180 độ

def DistanceMeasurement(TRIG,ECHO):
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
    Đường kính bánh xe: Dn = 95mm
    pi = 3.14
    Chu vi bánh xe: Dn*pi=95*3.14=298.3 (mm)
    Độ phân giải encoder 2400 xung/vòng
    Số mm/1xung=298.3/2400=0,1242916666666667 (mm)
"""
def Position(x_old, y_old, angle_old):
    # Nhận vị trí hiện tại
    positionA = myEncoderA.position
    positionB = myEncoderB.position
    # Pulse -> Distance
    wheel_A = positionA*(298.3/2400)
    print ("Distance_Wheel_A: ",wheel_A,"mm")
    wheel_B = positionB*(298.3/2400)
    print ("Distance_Wheel_B: ",wheel_B,"mm")

    b = 25      # Khoảng cánh giữa 2 bánh xe

    midpoint = (wheel_A + wheel_B) / 2        # Độ dịch chuyển tương đối của điểm trung tâm
    angle_new = (wheel_A + wheel_B) / b         # Góc xoay của xe

    angle_i = angle_old + angle_new       # Hướng tương đối của xe tại thời điểm i

    x_i = x_old + midpoint * math.cos(angle_i)         # Vị trí tương đối trục x của xe tại thời điểm i
    y_i = y_old + midpoint * math.sin(angle_i)         # Vị trí tương đối trục y của xe tại thời điểm i

    x_i, y_i, angle_i = x_old, y_old, angle_old 
    print ("x: %f" % x_i, "y: %f" % y_i, "angle: %f" % angle_i)

while True:
    Position(0, 0, 0)
    Control = input("Control DC Motor: ")
    if Control == "s":
        Stopmotor()
    if Control == "1":
        Forward()
    if Control == "2":
        Backward()
    if Control == "3":
        Right_Thuan()
    if Control == "4":
        Right_Nguoc()
    if Control == "5":
        Left_Thuan()
    if Control == "6":
        Left_Nguoc()



