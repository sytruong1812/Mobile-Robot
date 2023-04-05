import numpy as np

try:
    # Đọc dữ liệu từ file out.npy
    data = np.load('out.npy')

    # In kích thước của mảng dữ liệu
    print(data.shape)

    # In các giá trị của mảng dữ liệu
    print(data)
except:
    print("Không thể đọc được file out.npy!")
