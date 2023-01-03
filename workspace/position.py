import math

wheel_A = 60
wheel_B = 80

def position(x_old, y_old, angle_old):
    b = 30  # Khoảng cánh giữa 2 bánh xe
    global wheel_A
    global wheel_B
    
    midpoint = (wheel_A + wheel_B) / 2        # Độ dịch chuyển tương đối của điểm trung tâm
    angle_new = (wheel_A + wheel_B) / b         # Góc xoay của xe

    angle_i = angle_old + angle_new       # Hướng tương đối của xe tại thời điểm i

    x_i = x_old + midpoint * math.cos(angle_i)          # Vị trí tương đối trục x của xe tại thời điểm i
    y_i = y_old + midpoint * math.sin(angle_i)         # Vị trí tương đối trục y của xe tại thời điểm i
    print ("x: %f" % x_i, "y: %f" % y_i, "angle: %f" % angle_i)

while True:
    position(0, 0, 90)

x: 65.947953
y: 31.472079
angle: 94.666667