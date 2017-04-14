import threading
import time
import subprocess
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
        self.voiceQueueList = []
        for priority in xrange(constants.PRIORITIES.numberOf):
            self.voiceQueueList.append(Queue())
        self.lastProcess = None

    #################
    # Main voice loop
    #################
    def voiceLoop(self):
        while True:
            self.voiceLock.acquire()

            # Iterate through voice queues in order of priorities
            for voiceQueue in self.voiceQueueList:
                # If voice queue is not empty
                if not voiceQueue.empty():
                    # If no message is being spoken
                    if (self.lastProcess is None) or (self.lastProcess is not None and self.isProcessDone()):
                        # Get message from voice queue and speak
                        message = voiceQueue.get().message
                        self.sayMessage(message)
                    # Stop iterating through lower-priority voice queues
                    break;

            self.voiceLock.release()
            time.sleep(constants.SPEECH_DELAY)

    ##################
    # Helper functions
    ##################
    def addToQueue(self, instruction):
        self.voiceLock.acquire()
        # Select appropriate voice queue
        voiceQueue = self.voiceQueueList[instruction.priority]

        # Clear voice queue only for instructions that need to be up-to-date
        if instruction.priority in (
            constants.PRIORITIES.STEP_HIGH,
            constants.PRIORITIES.TURN,
            constants.PRIORITIES.STEP_LOW,
            ):
            voiceQueue.queue.clear()
        # Put instruction in appropriate queue regardless of priority
        voiceQueue.put(instruction)
        self.voiceLock.release()

    def sayMessage(self, message):
        print 'Voice Output: ' + message
        if not constants.IS_DEBUG_MODE:
            voiceCmd = ttsTemplate.format(msg = message)
            self.lastProcess = subprocess.Popen(voiceCmd, shell=True, stdout=subprocess.PIPE, preexec_fn=os.setsid)

    def isProcessDone(self):
        response = self.lastProcess.poll()
        if response is None:
            return False
        else:
            return True
