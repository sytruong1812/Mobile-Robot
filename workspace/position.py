def position(x_old, y_old, angle_old):
    # Nhận vị trí hiện tại
    positionA = myEncoderA.position
    positionB = myEncoderB.position
    # Pulse -> Distance
    wheel_A = positionA*(298.3/2400)
    print ("Distance_Wheel_A: ",wheel_A,"mm")
    wheel_B = positionB*(298.3/2400)
    print ("Distance_Wheel_B: ",wheel_B,"mm")

    b = 25      # Khoảng cánh giữa 2 bánh xe

    midpoint = (wheel_A + wheel_B) / 2        # Độ dịch chuyển tương đối của điểm trung tâm
    angle_new = (wheel_A + wheel_B) / b         # Góc xoay của xe

    angle_i = angle_old + angle_new       # Hướng tương đối của xe tại thời điểm i

    x_i = x_old + midpoint * math.cos(angle_i)         # Vị trí tương đối trục x của xe tại thời điểm i
    y_i = y_old + midpoint * math.sin(angle_i)         # Vị trí tương đối trục y của xe tại thời điểm i

    x_i, y_i, angle_i = x_old, y_old, angle_old 
    print ("x: %f" % x_i, "y: %f" % y_i, "angle: %f" % angle_i)