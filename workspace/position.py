import math
import random
import time

def position(x_old, y_old, angle_old):
    # Khởi tạo cho trước quãng đường đi được của 2 bánh xe
    wheel_A = random.randint(1, 100)
    wheel_B = random.randint(1, 100)
    b = 30  # Khoảng cánh giữa 2 bánh xe
    
    midpoint = (wheel_A + wheel_B) / 2        # Độ dịch chuyển tương đối của điểm trung tâm
    angle_new = (wheel_A + wheel_B) / b         # Góc xoay của xe

    angle_i = angle_old + angle_new       # Hướng tương đối của xe tại thời điểm i

    x_i = x_old + midpoint * math.cos(angle_i)          # Vị trí tương đối trục x của xe tại thời điểm i
    y_i = y_old + midpoint * math.sin(angle_i)         # Vị trí tương đối trục y của xe tại thời điểm i
    print ("x: %f" % x_i, "y: %f" % y_i, "angle: %f" % angle_i)
    time.sleep(3)
    x_old, y_old, angle_old = x_i, y_i, angle_i

while True:
    position(0, 0, 0)