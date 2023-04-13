import math
from firebase import firebase
import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.GPIO as GPIO
from Adafruit_BBIO.Encoder import RotaryEncoder, eQEP0, eQEP2
import pid_roirac_tocdo_phai
import pid_roirac_tocdo_trai

"""---------------------Firebase------------------------"""
firebase = firebase.FirebaseApplication('https://mobile-robot-6269b-default-rtdb.firebaseio.com/', None)

"""---------------------PWM-----------------------------"""
RPWM_A = "P8_13"
LPWM_A = "P8_19"
RPWM_B = "P9_16"
LPWM_B = "P9_14"
# PWM duty cycle. It must have a value from 0 to 100.
PWM.start(RPWM_A, 100, 2000, 1)
PWM.start(LPWM_A, 100, 2000, 1)
PWM.start(RPWM_B, 100, 2000, 1)
PWM.start(LPWM_B, 100, 2000, 1)
"""-------------------ENCODER----------------------------"""
myEncoderA = RotaryEncoder(eQEP2)       #eQEP0    P9.27    P9.42
myEncoderB = RotaryEncoder(eQEP0)       #eQEP2    P8.11    P8.12
# Chế độ tuyệt đối: vị trí bắt đầu từ 0 và được tăng hoặc giảm theo chuyển động của bộ mã hóa
myEncoderA.setAbsolute()
myEncoderB.setAbsolute()
pulseA = 0
pulseB = 0
"""----------------------Ultrasonic Sensor----------------------"""
GPIO.setup("P9_15",GPIO.OUT) #Trigger
GPIO.setup("P9_12",GPIO.IN)  #Echo
#Security
GPIO.output("P9_15", False)
# time.sleep(0.5)
"""--------------------------KINETICS----------------------------"""
R= 9,5/2      # Đường kính bánh xe D = 95mm -> R = D/2
d = 280     # Khoảng cách giữa 2 bánh xe
phi_right = 0   # Góc quay của bánh phải
phi_left = 0    # Góc quay của bánh trái
w_right = 0     # Vận tốc góc quay bánh phải
w_left = 0      # Vận tốc góc quay bánh trái
v = 0           # Vận tốc
theta = 0       # Góc đinh hướng của xe
x = 0           
y = 0

def Forward(pwm_right, pwm_left):
    PWM.set_duty_cycle(RPWM_A, pwm_left)
    PWM.set_duty_cycle(LPWM_A, 0)
    PWM.set_duty_cycle(RPWM_B, pwm_right)
    PWM.set_duty_cycle(LPWM_B, 0)
# Xe quay 1 góc 90 độ

# Xe quay ngược 1 góc 90 độ

# Xe quay 1 góc 180 độ

# Xe quay ngược 1 góc 180 độ

def kinetics():
    v = float(R * (w_right + w_left)) / 2
    theta = float(R * (w_right - w_left)) / d
    x = v * math.cos(theta)
    y = v * math.sin(theta)

def PID_right(gocquay_right, char):
    if char == 't':
        vitridat = gocquay_right
        pulseB = myEncoderB.position
        vitrithuc = pulseB * (360/2390)
        output = pid_roirac_tocdo_phai.PID(output, tocdodat)
        if vitrithuc < vitridat:
            tocdodat = tocdodat + 5
            if tocdodat >= 100: tocdodat = 100
            Forward()
def PID_left(gocquay_left, char):
    if char == 't':
        vitridat = gocquay_left
        pulseA = myEncoderB.position
        vitrithuc = pulseA * (360/2390)
        output = pid_roirac_tocdo_trai.PID(output, tocdodat)
        if vitrithuc < vitridat:
            tocdodat = tocdodat + 5
            if tocdodat >= 100: tocdodat = 100
            Forward()

"""
    Đường kính bánh xe: Dn = 95mm = 9.5cm
    pi = 3.14159
    Chu vi bánh xe: Dn*pi=9.5*3.14=29.83 (cm/vòng)
    Độ phân giải encoder 2400 xung/vòng
    Số cm/xung=29.83/2400=0.01242916667 (cm/xung)
    -> Xe đi 1m = 100cm cần: 100/(0.01242916666) = 8045 (xung)

    Hệ số khoảng cách đo bằng thực nghiệm cho xe chạy 1m, 2m, 3m
    đọc được số xung và tính ra hệ số giữa quãng đường đi và số xung
"""

def controlForward(distance):
    PPR = 2390
    K = 0.01275     # Hệ số khoảng cách
    pulse_right = distance / K
    pulse_left  = distance  / K

    gocquay_right = (pulse_right*360)/PPR
    gocquay_left  = (pulse_left*360)/PPR
    PID_right(gocquay_right, 't')
    PID_left(gocquay_left,'t')

def distanceForward(distance):
    PPR = 2390
    K = 0.01275     # Hệ số khoảng cách
    pulse_right = distance / K
    pulse_left  = distance  / K

    gocquay_right = (pulse_right*360)/PPR
    gocquay_left  = (pulse_left*360)/PPR

    pid_roirac_vitri_phai.PID(gocquay_left)
    pwm_left = pid_roirac_vitri_trai.output
    pid_roirac_vitri_trai.PID(gocquay_right)
    pwm_right = pid_roirac_vitri_phai.output

    Forward(pwm_left, pwm_right)
        
def main():
    while True:
        distanceForward(100)
if __name__ == "__main__":
    main()



        




