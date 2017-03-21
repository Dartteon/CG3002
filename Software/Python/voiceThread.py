#voiceThread.py
import threading
import time
import subprocess
# import signal
import os
from Queue import Queue

ttsTemplate = "espeak -s150 '{msg}' 2>/dev/null"

class VoiceHandler:
	def __init__(self):
		self.voiceLock = threading.Lock()
		self.voiceQueue = Queue()
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
				message = self.voiceQueue.get()
				self.sayMessage(message)
			elif self.lastProcess is not None and self.isProcessDone():
				message = self.voiceQueue.get()
				self.sayMessage(message)
			self.voiceLock.release()
			time.sleep(3)

	#########################
	# Helper functions
	#########################
	def addToQueue(self, message):
		self.voiceLock.acquire()
		if self.voiceQueue.empty():
			print 'queue empty.'
			self.voiceQueue.put(message)
			self.previousMessage = message
		else:
			self.voiceQueue.put(message)
			self.previousMessage = message
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
