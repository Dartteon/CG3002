#serial connection

import serial
import time

serial1 = serial.Serial('/dev/serial0',9600,timeout=1)

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
    print (readings)
