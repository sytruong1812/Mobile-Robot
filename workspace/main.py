import sys
import math
from firebase import firebase
import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.GPIO as GPIO
from Adafruit_BBIO.Encoder import RotaryEncoder, eQEP0, eQEP2
import pid_roirac_vitri_phai
import pid_roirac_vitri_trai

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

"""
    Đường kính bánh xe: Dn = 95mm = 9.5cm
    pi = 3.14159
    Chu vi bánh xe: Dn*pi=9.5*3.14=29.83 (cm/vòng)
    Độ phân giải encoder 2400 xung/vòng
    Số cm/xung=29.83/2400=0.01242916667 (cm/xung)
    -> Xe đi 1m = 100cm cần: 100/(0.01242916666) = 8045 (xung)
"""

def distanceForward():
    while True:  # Ví dụ về sử dụng vòng lặp while
        Forward(45, 45)
        pulseA = myEncoderA.position
        pulseB = myEncoderB.position
        if (pulseA == 7850) or (pulseB == 7850):
            Forward(0, 0)
            break  # Thoát khỏi vòng lặp while nếu điều kiện được thỏa mãn
        print ("PulseA: ", pulseA)
        print ("PulseB: ", pulseB)
        
def main():
    # distanceForward()
    while True:
        pid_roirac_vitri_phai.PID_roirac_vitri_phai(400, 0.03, 0.00005, 0.00005, 30)
        pwm_left = pid_roirac_vitri_trai.output
        pid_roirac_vitri_trai.PID_roirac_vitri_trai(400, 0.03, 0.00005, 0.00005, 30)
        pwm_right = pid_roirac_vitri_phai.output
        Forward(pwm_left, pwm_right)

if __name__ == "__main__":
    main()



        




