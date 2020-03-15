import serial 
import time
serial = serial.Serial("/dev/rfcomm0", 115200, timeout=1) 

olddata = 0
data = 0
while True:
	try:
		olddata = data
		data = serial.readline()
		data = str(data)
		if olddata == data:
			continue
		if data == "\r\n":
			continue
		if data == "":
			continue
		if data.find("#") == -1:
			continue
		if data.find("*") == -1:
			continue
		data = data.replace("*","")
		data = data.replace("#","")
		f = open('data.txt','w')
		f.write(data)
		f.close()
		print data	
		#break
	except:
		pass
