import serial
import time
import json
import traceback

class Arduino():
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
        # initialise serial port with Arduino
        # self.serial1 = serial.Serial('/dev/serial0',9600,timeout=0.01)
        if not self.serial1.isOpen():
            self.serial1.open()

    def serialWrite(self, message_str):
        self.ser.write(message_str)
        return

    def serialFlush(self):
        while self.serial1.inWaiting():
            readings = self.serial1.read()

    def serialRead(self):
        dataFlag = False
        #print ('start reading')
        while not dataFlag:
            serial1 = serial.Serial('/dev/serial0',9600,timeout=5)
            readings = serial1.readline()
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
                except:
                    traceback.print_exc()
                    #print(readings)
                serial1.write(msg)
                
                if msg == 'n':
                    print ('fail packet')
                    #print(readings) 
            return jsonO
            print('Timeout')

    def serial(self):
        return serial1
