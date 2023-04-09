import Adafruit_BBIO.PWM as PWM
from Adafruit_BBIO.Encoder import RotaryEncoder, eQEP0

RPWM_B = "P9_16"
LPWM_B = "P9_14"

PWM.start(RPWM_B, 100, 2000, 1)
PWM.start(LPWM_B, 100, 2000, 1)

myEncoderB = RotaryEncoder(eQEP0)       #eQEP0    P9.27    P9.42
myEncoderB.setAbsolute()
# myEncoderB.frequency = 1000
pulse = 0 

"""PID rời rạc vị trí"""
E = 0; E1 = 0; E2 = 0           # Sai số
output = 0; last_output = 0     # PWM điều khiển động cơ
alpha = 0; beta = 0; gama = 0 

Kp = 1; Ki = 0.0; Kd = 0.0
T = 30           # Thời gian lấy mẫu (Quan trọng)
PPR = 2300       # Encoder đo được 2390 xung / vòng

def PID_roirac_tocdo_phai(output, tocdodat, tocdo):
    global E, E1, E2, last_output
    global alpha, beta, gama, pulse
    
    # Tính sai số E
    if(tocdodat > tocdo): 
        E = abs(tocdodat) - abs(tocdo)
    if(tocdodat < tocdo): 
        E = abs(tocdo) - abs(tocdodat)
    # E = abs(tocdodat) - abs(tocdo)

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
        output = output
    return output

def Motor(pwm):
    PWM.set_duty_cycle(RPWM_B, pwm)
    PWM.set_duty_cycle(LPWM_B, 0)

try:
    while True:
        pulse = myEncoderB.position
        pulse = 0
        # tocdo = (pulse / 2300) * (1 / T) * 60        # Từ số xung/phút -> vận tốc
        tocdo = ((pulse) * (60 * (1000 / T))) / 2400        # Từ số xung/phút -> vận tốc

        output = PID_roirac_tocdo_phai(output, 50, tocdo)
        Motor(output)
        print("Ouput right:", output)
except KeyboardInterrupt:
    PWM.stop(RPWM_B)
    PWM.stop(LPWM_B)
    PWM.cleanup()



