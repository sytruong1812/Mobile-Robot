import Adafruit_BBIO.PWM as PWM
from Adafruit_BBIO.Encoder import RotaryEncoder, eQEP2

RPWM_A = "P8_13"
LPWM_A = "P8_19"

PWM.start(RPWM_A, 100, 2000, 1)
PWM.start(LPWM_A, 100, 2000, 1)

myEncoderA = RotaryEncoder(eQEP2)       #eQEP2    P8.11    P8.12
myEncoderA.setAbsolute()
# myEncoderB.frequency = 1000
pulse = 0 

"""PID rời rạc vị trí"""
tocdo = 0
E = 0; E1 = 0; E2 = 0           # Sai số
output = 0; last_output = 0     # PWM điều khiển động cơ
alpha = 0; beta = 0; gama = 0 

# Kp = 0.03; Ki = 0.00005; Kd = 0.00005
# T = 30           # Thời gian lấy mẫu (Quan trọng)
# PPR = 2300       # Encoder đo được 2390 xung / vòng

def PID_roirac_tocdo_trai(tocdodat, Kp, Kd, Ki, T):
    global E, E1, E2, last_output, output
    global alpha, beta, gama, pulse
    
    pulse = myEncoderA.position

    # tocdo = (pulse / 2300) * (60 / T)       # Từ số xung/phút -> vận tốc
    tocdo = ((pulse) * (60 * (1000 / T))) / 2300        # Từ số xung/phút -> vận tốc
    
    # Tính sai số E
    E = abs(tocdodat) - abs(tocdo)

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
        output = output
    return output

def Motor(pwm):
    PWM.set_duty_cycle(RPWM_A, pwm)
    PWM.set_duty_cycle(LPWM_A, 0)

try:
    while True:
        PID_roirac_tocdo_trai(60, 1.001, 0.0, 0.0, 30)
        print("Ouput left:", output)
        Motor(output)
except KeyboardInterrupt:
    PWM.stop(RPWM_A)
    PWM.stop(LPWM_A)
    PWM.cleanup()

