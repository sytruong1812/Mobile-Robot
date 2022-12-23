from Adafruit_BBIO.Encoder import RotaryEncoder, eQEP0, eQEP2

'''
ENCODER
Mỗi kênh có thể được truy cập và khởi tạo bằng các hằng số tên kênh tương ứng:
=======  =======  =======  ===================================================
Channel  Pin A    Pin B    Notes
=======  =======  =======  ===================================================
eQEP0    P9.27    P9.42    P9.92 alias for P9_42.1
eQEP1    P8.33    P8.35    Chỉ khả dụng khi tắt video
eQEP2    P8.11    P8.12    Chỉ khả dụng với eQEP2b chưa sử dụng (cùng kênh)
eQEP2b   P8.41    P8.42    Chỉ khả dụng khi tắt video và không sử dụng eQEP2
=======  =======  =======  ===================================================
'''
# Đường kính bánh xe: Dn = 95mm
# pi = 3.14
# Chu vi bánh xe: Dn*pi=95*3.14=298.3 (mm)
# Độ phân giải encoder 2400 xung/vòng
# Số mm/1xung=298.3/2400=0,1242916666666667 (mm)

# Khởi tạo lớp để truy cập kênh eQEP1, eQEP2 và chỉ khởi tạo kênh đó
myEncoderA = RotaryEncoder(eQEP0)
myEncoderB = RotaryEncoder(eQEP2)
# Chế độ tuyệt đối: vị trí bắt đầu từ 0 và được tăng hoặc giảm theo chuyển động của bộ mã hóa
myEncoderA.setAbsolute()
myEncoderB.setAbsolute()
while True:
    # Nhận vị trí hiện tại
    positionA = myEncoderA.position
    positionB = myEncoderB.position

    wheel_A = positionA*(198.3/2400)
    print ("Distance_Wheel_A: ",wheel_A,"mm")

    wheel_B = positionB*(298.3/2400)
    print ("Distance_Wheel_B: ",wheel_B,"mm")

    #print (cur_position)
    #print ("Position: %d" % cur_position)

    # Vị trí cũng có thể được đặt làm thuộc tính
    #next_position = 15
    #myEncoder.position = next_position

    # Đặt lại vị trí về 0
    #myEncoder.zero()

    # Thay đổi chế độ thành tương đối (mặc định là tuyệt đối)
    # Bạn có thể sử dụng setAbsolute() để chuyển về giá trị tuyệt đối
    # Tuyệt đối: vị trí bắt đầu từ 0 và được tăng hoặc giảm theo chuyển động của bộ mã hóa
    # Tương đối: vị trí được đặt lại khi bộ hẹn giờ tràn.
    #myEncoder.setRelative()

    # Đọc chế độ hiện tại (0: tuyệt đối, 1: tương đối)
    # Chế độ cũng có thể được đặt làm thuộc tính
    #mode = myEncoder.mode
    #print (mode)

    # Đọc tần suất cập nhật hiện tại
    # Giá trị trả về tính bằng Hz
    # Nếu bạn muốn đặt tần số, hãy chỉ định tần số bằng Hz
    #freq = myEncoder.frequency
    #print(freq)

    # Vô hiệu hóa kênh eQEP của bạn
    #myEncoder.disable()

    # Kiểm tra xem kênh đã được bật chưa
    # Thuộc tính 'đã bật' ở chế độ chỉ đọc
    # Sử dụng các phương thức enable() và disable() để
    # bật hoặc tắt mô-đun một cách an toàn
    #isEnabled = myEncoder.enabled