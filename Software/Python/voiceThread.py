#voiceThread.py
import threading
import time
import subprocess
# import signal
import os
from Queue import Queue
from Queue import PriorityQueue

ttsTemplate = "espeak -s150 '{msg}' 2>/dev/null"
SPEECH_DELAY = 1

class INSTRUCTION:
    '''Used instead of string to make comparable for priority queue'''
    def __init__(self, message, p):
        '''Initialise instruction'''
        self.message = str(message)
        self.p = int(p)
        
    def __lt__(self, other):
        '''Return true if self instruction has smaller p than other instruction'''
        return self.p < other.p

    def __eq__(self, other):
        '''Return true if both instructions have the same p'''
        return self.p == other.p
	
class VoiceHandler:
    def __init__(self):
        self.voiceLock = threading.Lock()
#         self.voiceQueue = Queue()
        self.voiceQueue = PriorityQueue()
        self.lastProcess = None
        self.previousMessage = ''

    #########################
    # Main voice loop
    #########################
    def voiceLoop(self):
        while True:
            self.voiceLock.acquire()
            if self.voiceQueue.empty():
                pass
            elif self.lastProcess is None:
                message = self.voiceQueue.get().message
                self.sayMessage(message)
            elif self.lastProcess is not None and self.isProcessDone():
                message = self.voiceQueue.get().message
                self.sayMessage(message)
            self.voiceLock.release()
            time.sleep(SPEECH_DELAY)

    #########################
    # Helper functions
    #########################
    def addToQueue(self, instruction):
        self.voiceLock.acquire()
        if self.voiceQueue.empty():
            print 'queue empty.'
            self.voiceQueue.put(instruction)
            self.previousMessage = instruction
        else:
            self.voiceQueue.put(instruction)
            self.previousMessage = instruction
        self.voiceLock.release()

    def sayMessage(self, message):
        print 'Voice output: ' + message
        voiceCmd = ttsTemplate.format(msg = message)
        self.lastProcess = subprocess.Popen(voiceCmd, shell=True, stdout=subprocess.PIPE, preexec_fn=os.setsid)

    def isProcessDone(self):
        response = self.lastProcess.poll()
        if response is None:
            return False
        else:
            return True
