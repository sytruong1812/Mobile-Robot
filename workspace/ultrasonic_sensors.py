import Adafruit_BBIO.GPIO as GPIO
import time

#Configuration
GPIO.setup("P9_15",GPIO.OUT) #Trigger
GPIO.setup("P9_12",GPIO.IN)  #Echo

#Security
GPIO.output("P9_15", False)
time.sleep(0.5)

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
    distance = pulseDuration * 17150        # multiply with the sonic speed (34300 cm/s)
    distance = round(distance, 2)   # Làm tròn số 
    return distance

while True:
    recoveredDistance = distanceMeasurement("P9_15","P9_12")
    print("Distance: ",recoveredDistance,"cm")
    time.sleep(1)
