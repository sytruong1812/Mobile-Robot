import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.GPIO as GPIO

#set polarity to 1 on start:
#PWM.start("P9_14", 50, 2000, 1)

#PWM.start(channel, duty, freq=2000, polarity=0)
#duty values are valid 0 (off) to 100 (on)

IN1 = "P8_7"
IN2 = "P8_8"

PWMA="P9_14"

GPIO.setup(IN1,GPIO.OUT)
GPIO.setup(IN2,GPIO.OUT)

PWM.start(PWMA, 100, 2000, 1)
while True:
    control = input()
    if control == '0':
        GPIO.output(IN1,GPIO.LOW)
        GPIO.output(IN2,GPIO.LOW)
        PWM.stop(PWMA)
    if control == '1':
        GPIO.output(IN1,GPIO.HIGH)
        GPIO.output(IN2,GPIO.LOW)
        PWM.set_duty_cycle(PWMA, 25)
    if control == '2':
        GPIO.output(IN1,GPIO.HIGH)
        GPIO.output(IN2,GPIO.LOW)
        PWM.set_duty_cycle(PWMA, 50)
    if control == '3':
        GPIO.output(IN1,GPIO.HIGH)
        GPIO.output(IN2,GPIO.LOW)
        PWM.set_duty_cycle(PWMA, 100)
