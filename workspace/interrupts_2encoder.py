import Adafruit_BBIO.GPIO as GPIO

"""Encoder"""
#Encoder 0
phaseA0 = "P9_27"
phaseB0 = "P9_42"
pulse0 = 0
GPIO.setup(phaseA0 , GPIO.IN, pull_up_down = GPIO.PUD_UP)
GPIO.setup(phaseB0 , GPIO.IN, pull_up_down = GPIO.PUD_UP)
GPIO.add_event_detect(phaseA0, GPIO.RISING)

#Encoder 2
phaseA2 = "P8_11"
phaseB2 = "P8_12"
pulse2 = 0
GPIO.setup(phaseA2 , GPIO.IN, pull_up_down = GPIO.PUD_UP)
GPIO.setup(phaseB2 , GPIO.IN, pull_up_down = GPIO.PUD_UP)
GPIO.add_event_detect(phaseA2, GPIO.RISING)

def Encoder0():
    global pulse0    # Khai báo biến toàn cục
    if GPIO.event_detected(phaseA0):
        if GPIO.input(phaseB0) == 0:
            pulse0 += 1
        else:
            pulse0 -= 1
    return pulse0
def Encoder2():
    global pulse2    # Khai báo biến toàn cục
    if GPIO.event_detected(phaseA2):
        if GPIO.input(phaseB2) == 0:
            pulse2 += 1
        else:
            pulse2 -= 1
    return pulse2

# while True:
#     Encoder1()
#     Encoder2()
#     print(f"pulse0={pulse0}, pulse2={pulse2}")