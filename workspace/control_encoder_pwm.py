import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.GPIO as GPIO
from Adafruit_BBIO.Encoder import RotaryEncoder, eQEP0, eQEP2

# Khởi tạo lớp để truy cập kênh eQEP1, eQEP2 và chỉ khởi tạo kênh đó
myEncoderA = RotaryEncoder(eQEP0)
myEncoderB = RotaryEncoder(eQEP2)
# Chế độ tuyệt đối: vị trí bắt đầu từ 0 và được tăng hoặc giảm theo chuyển động của bộ mã hóa
myEncoderA.setAbsolute()
myEncoderB.setAbsolute()

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

def Forward():
    PWM.set_duty_cycle(RPWM_A, 45)
    PWM.set_duty_cycle(LPWM_A, 0)
    PWM.set_duty_cycle(RPWM_B, 44)
    PWM.set_duty_cycle(LPWM_B, 0)
def Backward():
    PWM.set_duty_cycle(RPWM_A, 0)
    PWM.set_duty_cycle(LPWM_A, 45)
    PWM.set_duty_cycle(RPWM_B, 0)
    PWM.set_duty_cycle(LPWM_B, 45)
def Stopmotor():
    PWM.set_duty_cycle(RPWM_A, 0)
    PWM.set_duty_cycle(LPWM_A, 0)
    PWM.set_duty_cycle(RPWM_B, 0)
    PWM.set_duty_cycle(LPWM_B, 0)

def encoder():
    # Nhận vị trí hiện tại
    positionA = myEncoderA.position
    positionB = myEncoderB.position

    wheel_A = positionA*(298.3/2000)
    print ("Distance_Wheel_A: ",wheel_A,"mm")

    wheel_B = positionB*(298.3/2000)
    print ("Distance_Wheel_B: ",wheel_B,"mm")

    return wheel_A, wheel_B

while True:
    # Đặt lại vị trí về 0
    myEncoderA.zero()
    myEncoderB.zero()

    Control = input("Control DC Motor: ")

    if Control == "s":
        Stopmotor()
    if Control == "1":
        Forward()
        wheel_A, wheel_B = encoder()
        if encoder(wheel_A == 350.0 & wheel_B == 350.0):
            Stopmotor()
    if Control == "2":
        Backward()
        wheel_A, wheel_B = encoder()
        if encoder(wheel_A == -350.0 & wheel_B == -350.0):
            Stopmotor()


