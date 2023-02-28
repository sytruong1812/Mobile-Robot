import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.GPIO as GPIO
from pid_controller import PID

"""Encoder"""
# Khởi tạo lớp để truy cập kênh eQEP1, eQEP2 và chỉ khởi tạo kênh đó
myEncoderA = RotaryEncoder(eQEP0)
myEncoderB = RotaryEncoder(eQEP2)
# Chế độ tuyệt đối: vị trí bắt đầu từ 0 và được tăng hoặc giảm theo chuyển động của bộ mã hóa
myEncoderA.setAbsolute()
myEncoderB.setAbsolute()

"""PWM"""
RPWM_A = "P9_16"
LPWM_A = "P9_14"

RPWM_B = "P8_13"
LPWM_B = "P8_19"

#PWM duty cycle. It must have a value from 0 to 100.
PWM.start(RPWM_A, 100, 2000, 1)
PWM.start(LPWM_A, 100, 2000, 1)

PWM.start(RPWM_B, 100, 2000, 1)
PWM.start(LPWM_B, 100, 2000, 1)

"""PID"""

# Khởi tạo PID cho động cơ 1
pid1 = PID(p=1, i=0.1, d=0.05)
pid1.setpoint = 45
pid1.sample_time = 0.01
pid1.output_limits = (0, 100)  # Giới hạn điều khiển từ -0% đến 100%

# Khởi tạo PID cho động cơ 2
pid2 = PID(p=1, i=0.1, d=0.05)
pid2.setpoint = 45
pid2.sample_time = 0.01
pid2.output_limits = (0, 100)

while True:
    # Nhận vị trí hiện tại
    pulse_A = myEncoderA.position
    pulse_B = myEncoderB.position

    # Lấy giá trị tốc độ thực tế của động cơ 1
    speed1 = pulse_A * (2400/60)  # Đọc giá trị từ cảm biến hoặc bộ giả lập

    # Tính toán tín hiệu điều khiển cho động cơ 1
    control_signal1 = pid1(speed1)

    # Lấy giá trị tốc độ thực tế của động cơ 2
    speed2 = pulse_B * (2400/60)

    # Tính toán tín hiệu điều khiển cho động cơ 2
    control_signal2 = pid2(speed2)

    # Điều khiển tốc độ động cơ 1 bằng PWM
    if control_signal1 < 0:
        # Đổi chiều quay động cơ
        PWM.set_duty_cycle(RPWM_A, -control_signal1)
        PWM.set_duty_cycle(LPWM_A, 0)
    else:
        PWM.set_duty_cycle(RPWM_A, 0)
        PWM.set_duty_cycle(LPWM_A, control_signal1)

    # Điều khiển tốc độ động cơ 2 bằng PWM
    if control_signal2 < 0:
        # Đổi chiều quay động cơ
        PWM.set_duty_cycle(RPWM_B, -control_signal2)
        PWM.set_duty_cycle(LPWM_B, 0)
    else:
        PWM.set_duty_cycle(RPWM_B, 0)
        PWM.set_duty_cycle(LPWM_B, control_signal2)


