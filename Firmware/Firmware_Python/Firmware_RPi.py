#serial connection

import serial
import time
import json

serial1 = serial.Serial('/dev/serial0',9600,timeout=0.01)

initFlag = True

while initFlag:
    msg = "h"
	serial1.write(msg)
    msg = serial1.read()
    print(msg)
    if (msg == "a"):
        msg2 = 'a';
        serial1.write(msg2)
        initFlag=False
    


while True:
    readings = serial1.readline()
	if readings:
		msg ='n'
		print (readings)
		json0 = json.loads(readings)
		checksumPi = 0;
		for data in json0['timestamp']:
			checksumPi = (checksumPi + data) % 256
			if (checksumPi ==json0['checksum']):
				msg = 'a'
			print (json0['checksum'])
			print (checksumPi)
			serial1.write(msg)
