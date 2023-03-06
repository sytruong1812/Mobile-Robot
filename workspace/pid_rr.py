import Adafruit_BBIO.PWM as PWM
from Adafruit_BBIO.Encoder import RotaryEncoder, eQEP0

RPWM = "P9_16"
LPWM = "P9_14"

PWM.start(RPWM, 100, 2000, 1)
PWM.start(LPWM, 100, 2000, 1)

myEncoder = RotaryEncoder(eQEP0)
myEncoder.setAbsolute()

output = 0; tocdodat = 40; tocdothuc = 0

def PID_roirac_vantoc(output, tododat, tocdothuc):
    E = 0; E1 = 0; E2 = 0       # Sai số

    last_output = 0     # PWM điều khiển động cơ

    Kp= 5; Ki = 0; Kd = 0
    alpha = 0   
    beta = 0   
    gama = 0    
    T = 0.01       # Thời gian lấy mẫu (Quan trọng)
    PPR = 2390       # Encoder đo được 2390 xung / vòng

    pulse = myEncoder.position

    tocdothuc = (pulse / PPR) * (1/T) * 60
    pulse = 0 
    # E = tocdodat - tocdothuc
    if (tocdodat > tocdothuc): E = abs(tocdodat) - abs(tocdothuc)
    if (tocdodat < tocdothuc): E = abs(tocdothuc) - abs(tocdodat)

    alpha = (2 * T * Kp) + (Ki * T * T) + (2 * Kd)
    beta = (T * T * Ki) - (4 * Kd) - (2 * T * Kp)
    gama = 2 * Kd
    output = (alpha*E + beta*E1 + gama*E2 + 2*T*last_output) / (2*T)
    last_output = output
    E2 = E1
    E1 = E
    if(output > 60):
        output = 60
    if(output < 0):
        output = 0
    if(output > 0):
        PWM.set_duty_cycle(RPWM, output)
        PWM.set_duty_cycle(LPWM, 0)
    else:
        PWM.set_duty_cycle(RPWM, 0)
        PWM.set_duty_cycle(LPWM, 0)
    print(output)

while True:
    
    PID_roirac_vantoc(output, tocdodat, tocdothuc)