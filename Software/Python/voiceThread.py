#voiceThread.py
import threading
import time
import subprocess
# import signal
import os
import constants
from Queue import Queue

ttsTemplate = "espeak -s 150 -v en+f3 '{msg}' 2>/dev/null"
stepsSubstring = "steps"

class INSTRUCTION:
    '''Used instead of string to make comparable for priority queue'''
    def __init__(self, message, priority):
        '''Initialise instruction'''
        self.message = str(message)
        self.priority = int(priority)

    def __lt__(self, other):
        '''Return true if self instruction has smaller p than other instruction'''
        return self.priority < other.priority

    def __eq__(self, other):
        '''Return true if both instructions have the same p'''
        return self.priority == other.priority

class VoiceHandler:
    def __init__(self):
        self.voiceLock = threading.Lock()
#         self.voiceQueue = Queue()
        self.voiceQueue = Queue()
        self.lastProcess = None
        self.previousInstruction = INSTRUCTION('', constants.USELESS)

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
            time.sleep(constants.SPEECH_DELAY)

    #########################
    # Helper functions
    #########################
    def addToQueue(self, instruction):
        self.voiceLock.acquire()
        if self.voiceQueue.empty():
            print 'queue empty.'
            self.voiceQueue.put(instruction)
            self.previousInstruction = instruction
        elif self.previousInstruction.priority == instruction.priority and self.previousInstruction.message != instruction.message:
            # Check if the instruction queue contains the word "steps" in it
            # If so, clear the previous queue and push only the LATEST step guide command
            if stepsSubstring in previousInstruction.message and stepsSubstring in instruction.message:
                self.voiceQueue.queue.clear()
                self.voiceQueue.put(instruction)
                self.previousInstruction = instruction
            else:
                self.voiceQueue.put(instruction)
                self.previousInstruction = instruction
        elif self.previousInstruction.priority > instruction.priority and self.previousInstruction.message != instruction.message:
            self.voiceQueue.queue.clear()
            self.voiceQueue.put(instruction)
            self.previousInstruction = instruction
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
