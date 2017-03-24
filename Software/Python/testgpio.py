import os
from gpio import Keypad
keypad
def gpioMain():
    global keypad
    numberString = ''
    keypad = KeyPad()
    while numberString is not '0000':
        if numberString is not '':
            print(str(numberString))
            text_to_speech(str(numberString))
        numberString = keypad.getKeysInput()

def text_to_speech(text):
    os.system("espeak -s 200 -v en+f3 '{msg}' 2>/dev/null".format(msg = text))

gpioMain()
