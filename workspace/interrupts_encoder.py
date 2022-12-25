import Adafruit_BBIO.GPIO as GPIO

phaseA = "P8_11"
phaseB = "P8_12"

GPIO.setup(phaseA , GPIO.IN, pull_up_down = GPIO.PUD_UP)
GPIO.setup(phaseB , GPIO.IN, pull_up_down = GPIO.PUD_UP)

count = 0
GPIO.add_event_detect(phaseA, GPIO.RISING)

while True:
    if GPIO.event_detected(phaseA):
        if GPIO.input(phaseB) == 0:
            count += 1
        else:
            count -= 1
    print (count)
