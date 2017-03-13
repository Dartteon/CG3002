import serial
import time
import json


class Arduino():
    def handshakeWithArduino(self):
        return True

class SerialCommunicator():
    def __init__(self):
        # initialise serial port with Arduino
        #self.ser = serial.Serial('/dev/ttyAMA0', 9600)
        #self.ser.open()
        i=1

    def serialWrite(self, message_str):
        self.ser.write(message_str)
        return

    def serialRead(self):
        readings = "{'dir':[1,2,3,4,5],'accelx':[1,2,3,4,5],'accely':[1,2,3,4,5],'accelz':[1,2,3,4,5],'timestamp':[1,2,3,4,5],'checksum':15}"
        json0 = json.loads(readings)
        print(jsonO)
        return json0