import Adafruit_BBIO.PWM as PWM
from Adafruit_BBIO.Encoder import RotaryEncoder, eQEP0

RPWM_B = "P9_16"
LPWM_B = "P9_14"

PWM.start(RPWM_B, 100, 2000, 1)
PWM.start(LPWM_B, 100, 2000, 1)

myEncoderB = RotaryEncoder(eQEP0)       #eQEP0    P9.27    P9.42
myEncoderB.setAbsolute()

# pulse = 0 

"""PID rời rạc vị trí"""
vitri = 0
E = 0; E1 = 0; E2 = 0           # Sai số
output = 0; last_output = 0     # PWM điều khiển động cơ
alpha = 0; beta = 0; gama = 0 

# Kp = 0.03; Ki = 0.00005; Kd = 0.00005
# T = 30           # Thời gian lấy mẫu (Quan trọng)
# PPR = 2300       # Encoder đo được 2390 xung / vòng

def PID_roirac_vitri_phai(vitridat, Kp, Ki, Kd, T):
    global E, E1, E2, last_output, output
    global alpha, beta, gama
    
    pulse = myEncoderB.position

    vitri = ((pulse*360)/2300)
    
    # Tính sai số E
    E = vitridat - vitri

    alpha = (2 * T * Kp) + (Ki * T * T) + (2 * Kd)
    beta = (T * T * Ki) - (4 * Kd) - (2 * T * Kp)
    gama = 2 * Kd
    output = (alpha*E + beta*E1 + gama*E2 + 2*T*last_output) / (2*T)
    last_output = output
    E2 = E1
    E1 = E 

    if(output > 60):
        output = 60
    if(output < 30):
        output = 30
    if(output > 0):
        output = output
    return output

def Motor(a):
    PWM.set_duty_cycle(RPWM_B, a)
    PWM.set_duty_cycle(LPWM_B, 0)

while True:
    PID_roirac_vitri_phai(660, 0.03, 0.00005, 0.00005, 30)
    print("Ouput left:", output)
    Motor(output)




