import Adafruit_BBIO.PWM as PWM
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

RPWM_B = "P9_16"
LPWM_B = "P9_14"

PWM.start(RPWM_B, 100, 2000, 1)
PWM.start(LPWM_B, 100, 2000, 1)

pulse = 0 

E = 0; E1 = 0; E2 = 0           # Sai số
output = 0; last_output = 0     # PWM điều khiển động cơ
alpha = 0; beta = 0; gama = 0 

Kp = 1.5; Ki = 0.0; Kd = 0.01
T = 5           # Thời gian lấy mẫu (Quan trọng)
PPR = 600       # Encoder đo được 2390 xung / vòng

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

def PID(output, tocdodat):
    global E, E1, E2, last_output
    global alpha, beta, gama, pulse

    pulse = Encoder0()
    # tocdothuc = (pulse / 2300) * (1 / T) * 60        # Từ số xung/phút -> vận tốc
    tocdothuc = (pulse/PPR) * (1 / T) * 60        # Từ số xung/phút -> vận tốc
    # pulse = 0
    
    # Tính sai số E
    if(tocdodat > tocdothuc): 
        E = abs(tocdodat) - abs(tocdothuc)
    if(tocdodat < tocdothuc): 
        E = abs(tocdothuc) - abs(tocdodat)
    # E = abs(tocdodat) - abs(tocdothuc)

    alpha = (2 * T * Kp) + (Ki * T * T) + (2 * Kd)
    beta = (T * T * Ki) - (4 * Kd) - (2 * T * Kp)
    gama = 2 * Kd
    output = (alpha*E + beta*E1 + gama*E2 + 2*T*last_output) / (2*T)
    last_output = output
    E1 = E 
    E2 = E1

    if(output > 100):
        output = 100
    if(output < 30):
        output = 0
    if(output > 0):
        output = abs(output)
    print(f"Ouput right: {output}  Pulse: {pulse}")
    return output

def Motor(pwm):
    PWM.set_duty_cycle(RPWM_B, pwm)
    PWM.set_duty_cycle(LPWM_B, 0)

try:
    while True:
        output = PID(output, 45)
        Motor(output)
except KeyboardInterrupt:
    PWM.stop(RPWM_B)
    PWM.stop(LPWM_B)
    PWM.cleanup()



