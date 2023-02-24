import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.GPIO as GPIO

#set polarity to 1 on start:
#PWM.start("P9_14", 50, 1000, 1)

#PWM.start(channel, duty, freq=1000, polarity=0)
#duty values are valid 0 (off) to 100 (on)

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

while True:
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

