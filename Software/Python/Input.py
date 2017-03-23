import sys
import select

def getInput():
    i,o,e = select.select([sys.stdin],[],[]) # No timeout, function returns input after enter is registered.
    for s in i:
        if s == sys.stdin:
            input = sys.stdin.readline()
            return str(input)
    return e
