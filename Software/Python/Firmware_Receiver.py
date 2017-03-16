import serial
import time
import json


class Arduino():
    def handshakeWithArduino(self):
        serial = SerialCommunicator()
        initFlag = True
        while initFlag:
            msg = "h"
            serial.serial1.write(msg)
            msg = serial.serial1.read()
            print(msg)
            if (msg == "a"):
                msg2 = 'a';
                serial.serial1.write(msg2)
                initFlag = False

class SerialCommunicator():
    serial1 = serial.Serial('/dev/serial0',9600,timeout=0.01)
    def __init__(self):
        # initialise serial port with Arduino
        # self.serial1 = serial.Serial('/dev/serial0',9600,timeout=0.01)
        if not self.serial1.isOpen():
            self.serial1.open()

    def serialWrite(self, message_str):
        self.ser.write(message_str)
        return

    def serialRead(self):
        dataFlag = False
        while not dataFlag:
            serial1 = serial.Serial('/dev/serial0',9600,timeout=0.01)
            readings = serial1.readline()
            if readings:
                msg = 'n'
                print(readings)
                json0 = json.loads(readings)
                checksumPi = 0;
                for data in json0['timestamp']:
                    checksumPi = (checksumPi + data) % 256
                    if (checksumPi == json0['checksum']):
                        msg = 'a'
                        dataFlag = True
                    print(json0['checksum'])
                    print(checksumPi)
                    serial1.write(msg)
                return json0

    def serial(self):
        return serial1