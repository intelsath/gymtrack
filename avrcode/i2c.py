import serial 
serial = serial.Serial("/dev/rfcomm0", 9600, timeout=1) 

olddata = 0
data = 0
while True:

	#uses the serial port
	olddata = data;
	data = serial.readline()
	data = str(data)
	if olddata == data:
		continue
	if data == "\r\n":
		continue
	print data
