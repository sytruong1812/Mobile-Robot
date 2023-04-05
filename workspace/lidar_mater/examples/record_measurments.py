'''Ghi lại các phép đo vào một tập tin nhất định. Ví dụ sử dụng:

$ ./record_measurments.py out.txt'''
import sys
from rplidar import RPLidar


PORT_NAME = 'COM4'
path = r"C:\Users\SY TRUONG\Downloads\DATN\workspace\lidar_mater\examples\out.txt"


def run(path):
    '''Main function'''
    lidar = RPLidar(PORT_NAME)
    outfile = open(path, 'w')
    try:
        print('Recording measurments... Press Crl+C to stop.')
        for measurment in lidar.iter_measures():
            line = '\t'.join(str(v) for v in measurment)
            outfile.write(line + '\n')
    except KeyboardInterrupt:
        print('Stoping.')
    lidar.stop()
    lidar.disconnect()
    outfile.close()

if __name__ == '__main__':
    run(path)
