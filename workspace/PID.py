import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.GPIO as GPIO
from pid_controller import PID

setpoint1 = 45
setpoint2 = 45

# Khởi tạo PID cho động cơ 1
pid1 = PID(p=1, i=0.1, d=0.05)
pid1.setpoint = setpoint1
pid1.sample_time = 0.01
pid1.output_limits = (-100, 100)  # Giới hạn điều khiển từ -100% đến 100%

# Khởi tạo PID cho động cơ 2
pid2 = PID(p=1, i=0.1, d=0.05)
pid2.setpoint = setpoint2
pid2.sample_time = 0.01
pid2.output_limits = (-100, 100)

# Lấy giá trị tốc độ thực tế của động cơ 1
speed1 = ...  # Đọc giá trị từ cảm biến hoặc bộ giả lập

# Tính toán tín hiệu điều khiển cho động cơ 1
control_signal1 = pid1(speed1)

# Lấy giá trị tốc độ thực tế của động cơ 2
speed2 = ...

# Tính toán tín hiệu điều khiển cho động cơ 2
control_signal2 = pid2(speed2)

