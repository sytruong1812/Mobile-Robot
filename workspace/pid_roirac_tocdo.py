import Adafruit_BBIO.PWM as PWM
from Adafruit_BBIO.Encoder import RotaryEncoder, eQEP0

RPWM = "P9_16"
LPWM = "P9_14"

PWM.start(RPWM, 100, 2000, 1)
PWM.start(LPWM, 100, 2000, 1)

myEncoder = RotaryEncoder(eQEP0)
myEncoder.setAbsolute()

tocdodat = 60; tocdo = 0
E = 0; E1 = 0; E2 = 0       # Sai số
output = 0; last_output = 0     # PWM điều khiển động cơ

alpha = 0   
beta = 0   
gama = 0 
Kp = 1.001; Ki = 0.0; Kd = 0.0
T = 30           # Thời gian lấy mẫu (Quan trọng)
PPR = 2300       # Encoder đo được 2390 xung / vòng

def PID_roirac_vantoc():
    global E, E1, E2, last_output, output
    global alpha, beta, gama, pulse
    
    pulse = myEncoder.position
 
    # tocdo = (pulse / 2300) * (60 / T)       # Từ số xung/phút -> vận tốc
    tocdo = (pulse / 60) / (2300 * T)        # Từ số xung/phút -> vận tốc
    # tocdo = (pulse * T) / (2300 * 2000)    # Từ số xung/phút -> vận tốc
    pulse = 0
    
    # Tính sai số E
    E = tocdodat - tocdo
    
    alpha = (2 * T * Kp) + (Ki * T * T) + (2 * Kd)
    beta = (T * T * Ki) - (4 * Kd) - (2 * T * Kp)
    gama = 2 * Kd
    output = (alpha*E + beta*E1 + gama*E2 + 2*T*last_output) / (2*T)
    last_output = output
    E2 = E1
    E1 = E 

    if(output > 100):
        output = 100
    if(output < 0):
        output = 0
    if(output > 0):
        output = output
    return output

def Motor(a):
    PWM.set_duty_cycle(RPWM, a)
    PWM.set_duty_cycle(LPWM, 0)

while True:
    PID_roirac_vantoc()
    print("Ouput:", output)
    Motor(output)



