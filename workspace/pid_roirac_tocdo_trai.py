import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.GPIO as GPIO
# from Adafruit_BBIO.Encoder import RotaryEncoder, eQEP2

#Encoder 2
phaseA2 = "P8_11"
phaseB2 = "P8_12"
pulse2 = 0
GPIO.setup(phaseA2 , GPIO.IN, pull_up_down = GPIO.PUD_UP)
GPIO.setup(phaseB2 , GPIO.IN, pull_up_down = GPIO.PUD_UP)
GPIO.add_event_detect(phaseA2, GPIO.RISING)

# myEncoderA = RotaryEncoder(eQEP2)       #eQEP2    P8.11    P8.12
# myEncoderA.setAbsolute()
# myEncoderB.frequency = 1000

RPWM_A = "P8_13"
LPWM_A = "P8_19"

PWM.start(RPWM_A, 100, 2000, 1)
PWM.start(LPWM_A, 100, 2000, 1)

E = 0; E1 = 0; E2 = 0           # Sai số
output = 0; last_output = 0     # PWM điều khiển động cơ
alpha = 0; beta = 0; gama = 0 

Kp = 1.5; Ki = 0.0; Kd = 0.01
T = 5           # Thời gian lấy mẫu (Quan trọng)
PPR = 600       # Encoder đo được 2390 xung / vòng

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
    
    # pulse = myEncoderA.position
    pulse = Encoder2()

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
    E2 = E1
    E1 = E 

    if(output > 100):
        output = 100
    if(output < 30):
        output = 30
    if(output > 0):
        output = abs(output)
    print(f"Ouput left: {output}  Pulse: {pulse}")
    return output

def Motor(pwm):
    PWM.set_duty_cycle(RPWM_A, pwm)
    PWM.set_duty_cycle(LPWM_A, 0)

try:
    while True:
        output = PID(output, 45)
        Motor(output)
except KeyboardInterrupt:
    PWM.stop(RPWM_A)
    PWM.stop(LPWM_A)
    PWM.cleanup()


