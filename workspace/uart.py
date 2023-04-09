import serial

'''
dataSen = 5byte
 - Quality
 - Angle
 - Distance
'''
stopPacket = {'A5', '25'}; 		#Stop request
resetPacket = {'A5', '40'};		#Reset request

ser = serial.Serial(port = "/dev/ttyO1", baudrate = 115200)		# UART1: /dev/ttyO1, Rx: P9_26, Tx: P9_24
ser.close()
ser.open()
if ser.isOpen():
	print("Serial is open!")
ser.write(stopPacket)
ser.read(size = 1)		#1byte = 8bit (Start flag|Quality|Angle|Distance|Checksum)
ser.close()

# Eventually, you'll want to clean up, but leave this commented for now, 
# as it doesn't work yet
#UART.cleanup()