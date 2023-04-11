import math

R = 9,5/2      # Đường kính bánh xe D = 95mm
d = 280     # Khoảng cách giữa 2 bánh xe

phi_right = 0   # Góc quay của bánh phải
phi_left = 0    # Góc quay của bánh trái
w_right = 0     # Vận tốc góc quay bánh phải
w_left = 0      # Vận tốc góc quay bánh trái
v = 0           # Vận tốc
theta = 0       # Góc đinh hướng của xe
x = 0           
y = 0

v = float(R * (w_right + w_left)) / 2

theta = float(R * (w_right - w_left)) / d
x = v * math.cos(theta)
y = v * math.sin(theta)