#voiceThread.py
import threading
import time
import subprocess
# import signal
import os
from Queue import Queue
from Queue import PriorityQueue

ttsTemplate = "espeak -s150 '{msg}' 2>/dev/null"
SPEECH_DELAY = 0.5
speech_triggers = [False, False, False, False]
currStepCount = 0
currTurnAngle = 0
currReachedNode = 0
currWalkAmount = 0

class INSTRUCTION:
    '''Used instead of string to make comparable for priority queue'''
    def __init__(self, m, p, t):
        '''Initialise instruction'''
        self.m = str(m)
        self.p = int(p)
        self.t = float(t)

    def __lt__(self, other):
        '''Return true if self instruction has smaller p than other instruction'''
        if self.p == other.p:
            return self.t < other.t
        return self.p < other.p

    def __eq__(self, other):
        '''Return true if both instructions have the same p'''
        if self.p == other.p:
            return self.t == other.t
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
            message = ''
            if (self.voiceQueue.empty()):
                if (speech_triggers[0]):
                    speech_triggers[0] = False
                    message = str(currStepCount) + " Steps Taken"
                elif (speech_triggers[1]):
                    speech_triggers[1] = False
                    message = "Reached Node " + str(currReachedNode)
                elif (speech_triggers[2]):
                    speech_triggers[2] = False
                    message = "Turn " + str(currTurnAngle) + " Degrees"
                elif (speech_triggers[3]):
                    message = "Walk " + str(currWalkAmount)
                    speech_triggers[3] = False

            if message is not '':
                print(message)
                if self.lastProcess is None:
                    self.sayMessage(message)
                elif self.lastProcess is not None and self.isProcessDone():
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

    def setSpeechTriggerStep(stepCount):
        print 'trigger step activated' + str(stepCount)
        currStepCount = stepCount
        speech_triggers[0] = True

    def setSpeechTriggerNode(node):
        currReachedNode = node
        speech_triggers[1] = True

    def setSpeechTriggerTurn(angle, test):
        currTurnAngle = angle
        speech_triggers[2] = True

    def setSpeechTriggerWalk(dist):
        currWalkAmount = dist
        speech_triggers[3] = True

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
