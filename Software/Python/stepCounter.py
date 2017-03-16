from datetime import datetime
from datetime import timedelta
import math #generating randomised data for test purposes
import random #generating randomised data for test purposes

MINIMUM_ACCELERATION_DELTA = 200 #200
MINIMUM_STEP_INTERVAL_MILLISECONDS = 800 #800
MINIMUM_ACCELERATION_Z = 1500 #1500
NUM_SAMPLE_COUNTS_TO_RECALCULATE_THRESHOLD = 50
DIST_PER_STEP_CM = 75 
PREDEFINED_PRECISION = 550

currTime = 0
lastStepTime = 0
xDynamicThreshold = 0
xMax = -9999
xMin = 9999
currSampleCount = 0
numStepsTaken = 0
currGyroZ = 0
currGyroY = 0
timestamp = 0

def read_step_counter(x, z, timestamp):
	prevSample = 0
	global lastStepTime, numStepsTaken, xMax, xMin
	if x > xMax: xMax = x
	if x < xMin: xMin = x
  	incrementSampleCount(currSampleCount, xMax, xMin)
  	xAccDelta = abs(prevSample - x)
  	prevSample = x
	currTime = timestamp
	timeDiff = timestamp - lastStepTime

  	# if (xAccDelta <= MINIMUM_ACCELERATION_DELTA) return;
  	if xAccDelta < MINIMUM_ACCELERATION_DELTA: 
  		print('below minimum acceleration delta')
  		return  #Check that walker has accelerated significantly
  	if timeDiff < MINIMUM_STEP_INTERVAL_MILLISECONDS: 
  		# print('below minimum step interval')
  		return #Check that steps arent double counted
  	if z < MINIMUM_ACCELERATION_Z:
  		print('below minimum acceleration z')
  		return #Check that walker is accelerating forward
  	# xSamples[currSampleCount] = xSampleNew; //Not needed anymore, removal TBI

	if timeDiff >= MINIMUM_STEP_INTERVAL_MILLISECONDS:
		if x < xDynamicThreshold:
			lastStepTime = currTime
			numStepsTaken += 1
			totalDist = DIST_PER_STEP_CM * numStepsTaken
			print("Step taken! Total steps - " + str(numStepsTaken) + " ---- AccZ = " + str(z) + " total distance = " + str(totalDist))
		else:
			print('greater than dynamic threshold')
			# Serial.println("Step detected but not within interval threshold");
	else:
		print('greater than step interval')

def incrementSampleCount(currSampleCount, xMax, xMin):
	if currSampleCount >= NUM_SAMPLE_COUNTS_TO_RECALCULATE_THRESHOLD - 1:
		calculateNewXThreshold(xMax, xMin) #Set new threshold
		currSampleCount = 0
	else:
		currSampleCount += 1

def calculateNewXThreshold(xMax, xMin):
	xDynamicThreshold = (xMax + xMin) / 2
	# Serial.print("xDynamicThreshold = "); Serial.println(xDynamicThreshold);
	xMax = -9999
	xMin = 9999

def milli_time_diff(startTime, endTime):
	timeDiff = endTime - startTime
	timeDiffMillis = (timeDiff.days * 24 * 60 * 60 + timeDiff.seconds) * 1000 + timeDiff.microseconds / 1000.0
	return int(timeDiffMillis)

# Used for testing millisecond time lol
def fib(x):
	if x == 0 or x == 1:
		return 1
	else:
		return fib(x-1) + fib(x-2)

# currentTime = datetime.now()
# previousTime = 0
# for x in xrange(1,40):
# 	previousTime = currentTime
# 	currentTime = datetime.now()
# 	print(fib(x), milli_time_diff(previousTime, currentTime))

# for x in xrange(1,500):
# 	y = fib(22)
# 	xSampleNew = math.sin(x)
# 	compass['ax'] = -math.sin(x)
# 	compass['ay'] = -math.sin(x)
# 	compass['az'] = -math.sin(x)
# 	read_step_counter()	

def step_count_simulation():
	# random x, z and timestamp for simulation
	for i in range(1,1000):
		global timestamp
		timestamp += random.randint(0, 30)
		read_step_counter(random.randint(-10000, 10000), random.randint(-5000, 10000), timestamp)