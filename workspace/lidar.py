import math

distance = 30
angle = 12

x = distance * math.cos(math.radians(angle))
y = distance * math.sin(math.radians(angle))

print("X-coordinate:", x)
print("Y-coordinate:", y)