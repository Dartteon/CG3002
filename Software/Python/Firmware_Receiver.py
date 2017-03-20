import serial
import time
import json


class Arduino():
    serial1 = serial.Serial('/dev/serial0', 9600, timeout=2)
    def __init__(self):
        if not self.serial1.isOpen():
            self.serial1.open()

    def handshakeWithArduino(self):
        handshake = True
        print('Initializing Handshake Sequence')
        while handshake:
            msg = "h"
            serial1.write(msg)
            msg = serial1.read()
            print(msg)
            if (msg == "a"):
                msg2 = 'a';
                serial1.write(msg2)
                handshake = False
                print ('Handshake Succeed')
            else:
                print ('Handshake Fail')

class SerialCommunicator():
    serial1 = serial.Serial('/dev/serial0',9600,timeout=2)
    def __init__(self):
        if not self.serial1.isOpen():
            self.serial1.open()

    def serialRead(self):
        dataFlag = False
        while not dataFlag:
            serial1 = serial.Serial('/dev/serial0',9600,timeout=0.01)
            readings = serial1.readline()
            if readings:
                print(readings)
                msg = 'n'
                try:
                    json0 = json.loads(readings)
                    checksumPi = (jsonO['direction'] + int(jsonO['distance'])) % 256
                    for data in jsonO['timestamp']:
                        checksumPi = (checksumPi + data) % 256
                        if (checksumPi == jsonO['checksum']):
                            msg = 'a'
                            dataFlag = True
                        #print(json0['checksum'])
                        #print(checksumPi)
                except:
                    print('Error reading')
                serial1.write(msg)
                return json0
            #timeout, from reset
            print('Timeout')
            arduino = Arduino()
            arduino.handshake
