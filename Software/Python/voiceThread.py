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
        self.voiceQueue = Queue()
        self.voiceStepQueue = Queue()
        self.lastProcess = None
        self.previousInstruction = INSTRUCTION('', constants.USELESS)

    #########################
    # Main voice loop
    #########################
    # def voiceLoop(self):
    #     while True:
    #         self.voiceLock.acquire()
    #         if self.voiceQueue.empty():
    #             pass
    #         elif self.lastProcess is None:
    #             message = self.voiceQueue.get().message
    #             self.sayMessage(message)
    #         elif self.lastProcess is not None and self.isProcessDone():
    #             message = self.voiceQueue.get().message
    #             self.sayMessage(message)
    #         self.voiceLock.release()
    #         time.sleep(constants.SPEECH_DELAY)

    def voiceLoop(self):
        while True:
            self.voiceLock.acquire()
            # If step queue is empty
            if self.voiceStepQueue.empty():
                # If standard queue is empty
                if self.voiceQueue.empty():
                    pass
                elif self.lastProcess is None:
                    message = self.voiceQueue.get().message
                    self.sayMessage(message)
                elif self.lastProcess is not None and self.isProcessDone():
                    message = self.voiceQueue.get().message
                    self.sayMessage(message)
            elif self.lastProcess is None:
                stepMessage = self.voiceStepQueue.get().message
                self.sayMessage(stepMessage)
            elif self.lastProcess is not None and self.isProcessDone():
                stepMessage = self.voiceStepQueue.get().message
                self.sayMessage(stepMessage)
            self.voiceLock.release()
            time.sleep(constants.SPEECH_DELAY)

    #########################
    # Helper functions
    #########################
    # def addToQueue(self, instruction):
    #     self.voiceLock.acquire()
    #     if self.voiceQueue.empty():
    #         print 'queue empty.'
    #         self.voiceQueue.put(instruction)
    #         self.previousInstruction = instruction
    #     else:
    #         self.voiceQueue.queue.clear()
    #         self.voiceQueue.put(instruction)
    #         self.previousInstruction = instruction
    #     # elif self.previousInstruction.priority == instruction.priority and self.previousInstruction.message != instruction.message:
    #     #     self.voiceQueue.queue.clear()
    #     #     self.voiceQueue.put(instruction)
    #     #     self.previousInstruction = instruction
    #     # elif self.previousInstruction.priority > instruction.priority and self.previousInstruction.message != instruction.message:
    #     #     self.voiceQueue.queue.clear()
    #     #     self.voiceQueue.put(instruction)
    #     #     self.previousInstruction = instruction
    #     self.voiceLock.release()

    def addToQueue(self, instruction):
        self.voiceLock.acquire()
        if self.instruction.priority == constants.MED_PRIORITY:
            if self.voiceStepQueue.empty():
                print 'step queue empty.'
                self.voiceStepQueue.put(instruction)
                self.previousInstruction = instruction
            else:
                self.voiceStepQueue.queue.clear()
                self.voiceStepQueue.put(instruction)
                self.previousInstruction = instruction
        else:
            if self.voiceQueue.empty():
                print 'queue empty.'
                self.voiceQueue.put(instruction)
                self.previousInstruction = instruction
            elif self.previousInstruction.priority == instruction.priority and self.previousInstruction.message != instruction.message:
                self.voiceQueue.put(instruction)
                self.previousInstruction = instruction
            elif self.previousInstruction.priority > instruction.priority and self.previousInstruction.message != instruction.message:
                self.voieQueue.queue.clear()
                self.voiceQueue.put(instruction)
                self.previousInstruction = instruction
        self.voiceLock.release()

    def sayMessage(self, message):
        print 'Voice output: ' + message
        if not constants.IS_DEBUG_MODE:
            voiceCmd = ttsTemplate.format(msg = message)
            self.lastProcess = subprocess.Popen(voiceCmd, shell=True, stdout=subprocess.PIPE, preexec_fn=os.setsid)

    def isProcessDone(self):
        response = self.lastProcess.poll()
        if response is None:
            return False
        else:
            return True
