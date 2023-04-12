import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.GPIO as GPIO
from firebase import firebase

"""---------------------Firebase------------------------"""
firebase = firebase.FirebaseApplication('https://mobile-robot-6269b-default-rtdb.firebaseio.com/', None)

#set polarity to 1 on start:
#PWM.start("P9_14", 50, 1000, 1)

#PWM.start(channel, duty, freq=1000, polarity=0)
#duty values are valid 0 (off) to 100 (on)

"""PWM"""
RPWM_A = "P8_13"
LPWM_A = "P8_19"

RPWM_B = "P9_16"
LPWM_B = "P9_14"


#PWM duty cycle. It must have a value from 0 to 100.
PWM.start(RPWM_A, 100, 2000, 1)
PWM.start(LPWM_A, 100, 2000, 1)

PWM.start(RPWM_B, 100, 2000, 1)
PWM.start(LPWM_B, 100, 2000, 1)

def Forward(pwm_right, pwm_left):
    PWM.set_duty_cycle(RPWM_A, pwm_left)
    PWM.set_duty_cycle(LPWM_A, 0)
    PWM.set_duty_cycle(RPWM_B, pwm_right)
    PWM.set_duty_cycle(LPWM_B, 0)
def Backward(pwm_right, pwm_left):
    PWM.set_duty_cycle(RPWM_A, 0)
    PWM.set_duty_cycle(LPWM_A, pwm_left)
    PWM.set_duty_cycle(RPWM_B, 0)
    PWM.set_duty_cycle(LPWM_B, pwm_right)
def Right_Thuan(pwm_right, pwm_left):
    PWM.set_duty_cycle(RPWM_A, pwm_right)
    PWM.set_duty_cycle(LPWM_A, 0)
    PWM.set_duty_cycle(RPWM_B, 0)
    PWM.set_duty_cycle(LPWM_B, 0)
def Right_Nguoc(pwm_right, pwm_left):
    PWM.set_duty_cycle(RPWM_A, 0)
    PWM.set_duty_cycle(LPWM_A, pwm_right)
    PWM.set_duty_cycle(RPWM_B, 0)
    PWM.set_duty_cycle(LPWM_B, 0)
def Left_Thuan(pwm_right, pwm_left):
    PWM.set_duty_cycle(RPWM_A, 0)
    PWM.set_duty_cycle(LPWM_A, 0)
    PWM.set_duty_cycle(RPWM_B, pwm_left)
    PWM.set_duty_cycle(LPWM_B, 0)
def Left_Nguoc(pwm_right, pwm_left):
    PWM.set_duty_cycle(RPWM_A, 0)
    PWM.set_duty_cycle(LPWM_A, 0)
    PWM.set_duty_cycle(RPWM_B, 0)
    PWM.set_duty_cycle(LPWM_B, pwm_left)
def Stopmotor(pwm_right, pwm_left):
    PWM.set_duty_cycle(RPWM_A, pwm_right)
    PWM.set_duty_cycle(LPWM_A, pwm_right)
    PWM.set_duty_cycle(RPWM_B, pwm_left)
    PWM.set_duty_cycle(LPWM_B, pwm_left)

def Control(pwm_right, pwm_left):
    Control = input("Control DC Motor: ")
    if Control == '1':
        Forward(pwm_right, pwm_left)
    if Control == '2':
        Backward(pwm_right, pwm_left)
    if Control == '3':
        Right_Thuan(pwm_right, pwm_left)
    if Control == '4':
        Right_Nguoc(pwm_right, pwm_left)
    if Control == '5':
        Left_Thuan(pwm_right, pwm_left)
    if Control == '6':
        Left_Nguoc(pwm_right, pwm_left)
    if Control == '0':
        Stopmotor(0, 0)
    if Control == 'c':
        PWM.stop(LPWM_A)
        PWM.stop(RPWM_A)
        PWM.stop(LPWM_B)
        PWM.stop(RPWM_B)
        PWM.cleanup()

def controlFirebase():
    result = firebase.get('/Control', None)
    if result["Forward"] == 1:
        Forward(45, 45)
        print("Forward")
    elif result["Backward"] == 1:
        Backward(45, 45)
        print("Backward")
    elif result["Left_Thuan"] == 1:
        Left_Thuan(45, 45)
        print("Left_Thuận")
    elif result["Left_Nguoc"] == 1:
        Left_Nguoc(45, 45)
        print("Left_Ngược")
    elif result["Right_Thuan"] == 1:
        Right_Thuan(45, 45)
        print("Right_Thuận")
    elif result["Right_Nguoc"] == 1:
        Right_Nguoc(45, 45)
        print("Right_Ngược")
    else:
        Stopmotor(0, 0)
        print("Stopmotor")

while True:
        Control(45, 45)


