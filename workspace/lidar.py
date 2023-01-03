from pyrplidar import PyRPlidar
# Nguồn: https://pypi.org/project/pyrplidar/


lidar = PyRPlidar()
lidar.connect(port="COM6", baudrate=256000, timeout=3)
# Linux   : "/dev/ttyUSB0"
# MacOS   : "/dev/cu.SLAB_USBtoUART"
# Windows : "COM5"


info = lidar.get_info()
print("info :", info)

health = lidar.get_health()
print("health :", health)

samplerate = lidar.get_samplerate()
print("samplerate :", samplerate)


scan_modes = lidar.get_scan_modes()
print("scan modes :")
for scan_mode in scan_modes:
    print(scan_mode)


lidar.disconnect()