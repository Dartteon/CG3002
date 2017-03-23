import serial
import time
import json
import traceback

class Arduino():
    #deprecated
    def handshakeWithArduino(self):
        print ('start handshake')
        serial = SerialCommunicator()
        initFlag = True
        while initFlag:
            msg = "h"
            serial.serial1.write(msg)
            msg = serial.serial1.read()
            print(msg)
            if (msg == "a"):
                msg2 = 'a'
                serial.serial1.write(msg2)
                initFlag = False
        print ('end handshake')

class SerialCommunicator():
    serial1 = serial.Serial('/dev/serial0',9600,timeout=5)
    def __init__(self):
        if not self.serial1.isOpen():
            self.serial1.open()

    def serialFlush(self):
        while self.serial1.inWaiting():
            readings = self.serial1.read()

    def handshakeWithArduino(self):
        print ('Start handshake')
        initFlag = True
        while initFlag:
            msg = "h"
            self.serial1.write(msg)
            msg = self.serial1.read()
            print(msg)
            if (msg == "a"):
                msg2 = 'a'
                self.serial1.write(msg2)
                initFlag = False
                return
            print('Timeout Handshake')
        print ('End handshake')


    def serialRead(self):
        dataFlag = False
        while not dataFlag:
            readings = self.serial1.readline()
            if readings:
                msg = 'n'
                print(readings)
                try:
                    jsonO = json.loads(readings)
                    # print('step1')
                    checksumPi = 0
                    dir = jsonO["direction"]
                    # print(str(dir))
                    # print(jsonO['distance'])
                    checksumPi = (jsonO['direction'] + jsonO['distance'] ) % 256
                    # print('step2')
                    if (checksumPi == jsonO['checksum']):
                        msg = 'a'
                        dataFlag = True
                    print(jsonO['checksum'])
                    print(checksumPi)
                    self.serial1.write(msg)
                    return jsonO
                except:
                    traceback.print_exc()
                    #print(readings)
                self.serial1.write(msg)
                if msg == 'n':
                    print ('fail packet')
            print('Timeout serialRead')
			print('Initializing Handshake Sequence')
            self.handshakeWithArduino()

    def serial(self):
        return serial1
