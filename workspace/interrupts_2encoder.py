import Adafruit_BBIO.GPIO as GPIO

"""Encoder"""
#Encoder 1
phaseA1 = "P8_7"
phaseB1 = "P8_8"
count1 = 0
GPIO.setup(phaseA1 , GPIO.IN, pull_up_down = GPIO.PUD_UP)
GPIO.setup(phaseB1 , GPIO.IN, pull_up_down = GPIO.PUD_UP)
GPIO.add_event_detect(phaseA1, GPIO.RISING)
#Encoder 2
phaseA2 = "P8_11"
phaseB2 = "P8_12"
count2 = 0
GPIO.setup(phaseA2 , GPIO.IN, pull_up_down = GPIO.PUD_UP)
GPIO.setup(phaseB2 , GPIO.IN, pull_up_down = GPIO.PUD_UP)
GPIO.add_event_detect(phaseA2, GPIO.RISING)

def Encoder1():
    global count1    # Khai báo biến toàn cục
    if GPIO.event_detected(phaseA1):
        if GPIO.input(phaseB1) == 0:
            count1 += 1
        else:
            count1 -= 1
    print (count1)
def Encoder2():
    global count2    # Khai báo biến toàn cục
    if GPIO.event_detected(phaseA2):
        if GPIO.input(phaseB2) == 0:
            count2 += 1
        else:
            count2 -= 1
    print (count2)

while True:
    Encoder1()
    Encoder2()