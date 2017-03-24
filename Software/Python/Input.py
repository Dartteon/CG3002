import sys
import select
import os

def test_function():
    testString = ''
    while testString is not '0000':
        testString = get_input()
        text_to_speech(testString)

def get_input():
    i,o,e = select.select([sys.stdin],[],[], 0) # No timeout, function returns input after enter is registered.
    for s in i:
        if s == sys.stdin:
            input = sys.stdin.readline()
            return str(input)
    return e

def text_to_speech(text):
    os.system("espeak -s 200 -v en+f3 '{msg}' 2>/dev/null".format(msg = text))

test_function()
