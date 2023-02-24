import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.GPIO as GPIO

#set polarity to 1 on start:
#PWM.start("P9_14", 50, 2000, 1)

#PWM.start(channel, duty, freq=2000, polarity=0)
#duty values are valid 0 (off) to 100 (on)

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
PWMA = "P8_19"
PWMB = "P9_14"

#PWM duty cycle. It must have a value from 0 to 100.
PWM.start(PWMA, 100, 2000, 1)
PWM.start(PWMB, 100, 2000, 1)

def Forward():
    GPIO.output(IN1,GPIO.HIGH)
    GPIO.output(IN2,GPIO.LOW)
    GPIO.output(IN3,GPIO.HIGH)
    GPIO.output(IN4,GPIO.LOW)
    PWM.set_duty_cycle(PWMA, 25)
    PWM.set_duty_cycle(PWMB, 25)
def Backward():
    GPIO.output(IN1,GPIO.LOW)
    GPIO.output(IN2,GPIO.HIGH)
    GPIO.output(IN3,GPIO.LOW)
    GPIO.output(IN4,GPIO.HIGH)
    PWM.set_duty_cycle(PWMA, 25)
    PWM.set_duty_cycle(PWMB, 25)
def Right_Thuan():
    GPIO.output(IN1,GPIO.LOW)
    GPIO.output(IN2,GPIO.HIGH)
    GPIO.output(IN3,GPIO.LOW)
    GPIO.output(IN4,GPIO.LOW)
    PWM.set_duty_cycle(PWMA, 0)
    PWM.set_duty_cycle(PWMB, 30)
def Right_Nguoc():
    GPIO.output(IN1,GPIO.HIGH)
    GPIO.output(IN2,GPIO.LOW)
    GPIO.output(IN3,GPIO.LOW)
    GPIO.output(IN4,GPIO.LOW)
    PWM.set_duty_cycle(PWMA, 0)
    PWM.set_duty_cycle(PWMB, 100)
def Left_Thuan():
    GPIO.output(IN1,GPIO.LOW)
    GPIO.output(IN2,GPIO.LOW)
    GPIO.output(IN3,GPIO.HIGH)
    GPIO.output(IN4,GPIO.LOW)
    PWM.set_duty_cycle(PWMA, 100)
    PWM.set_duty_cycle(PWMB, 0)
def Left_Nguoc():
    GPIO.output(IN1,GPIO.LOW)
    GPIO.output(IN2,GPIO.LOW)
    GPIO.output(IN3,GPIO.LOW)
    GPIO.output(IN4,GPIO.HIGH)
    PWM.set_duty_cycle(PWMA, 100)
    PWM.set_duty_cycle(PWMB, 0)
def Stopmotor():
    GPIO.output(IN1,GPIO.LOW)
    GPIO.output(IN2,GPIO.LOW)
    GPIO.output(IN3,GPIO.LOW)
    GPIO.output(IN4,GPIO.LOW)
    PWM.set_duty_cycle(PWMA, 0)
    PWM.set_duty_cycle(PWMB, 0)

while True:
    Control = input("Control DC Motor: ")
    if Control == "s":
        Stopmotor()
    if Control == "1":
        Forward()
    if Control == "2":
        Backward()
    if Control == "3":
        Left_Thuan()
    if Control == "4":
        Right_Thuan()
    if Control == "5":
        Left_Nguoc()
    if Control == "6":
        Right_Nguoc()
    else:
        PWM.stop(PWMA)
        PWM.stop(PWMB)
        PWM.cleanup()
