import math
from datetime import datetime
from datetime import timedelta

MINIMUM_ACCELERATION_DELTA = 0 #200
MINIMUM_STEP_INTERVAL_MILLISECONDS = 800 #800
MINIMUM_ACCELERATION_Z = 0 #1500
NUM_SAMPLE_COUNTS_TO_RECALCULATE_THRESHOLD = 50
DIST_PER_STEP_CM = 75 
PREDEFINED_PRECISION = 550

currTime = 0
lastStepTime = datetime.now()
xDynamicThreshold = 0
xMax = -9999
xMin = 9999
currSampleCount = 0
xSampleNew = 0
numStepsTaken = 0
xAccOffset = 0
yAccOffset = 0
zAccOffset = 0
xFilter = [0] * 4
compass = {'ax': 0, 'ay': 0, 'az':0}
currGyroZ = 0
currGyroY = 0

def read_step_counter():
	global lastStepTime, numStepsTaken
	prevSample = xSampleNew #Get previous reading from  arduino
  	readAltimu()
  	xAccDelta = abs(prevSample - xSampleNew)
	currTime = datetime.now()
	timeDiff = milli_time_diff(lastStepTime, currTime)

  	# if (xAccDelta <= MINIMUM_ACCELERATION_DELTA) return;
  	if xAccDelta < MINIMUM_ACCELERATION_DELTA: 
  		print('minimum acceleration delta')
  		return  #Check that walker has accelerated significantly
  	if timeDiff < MINIMUM_STEP_INTERVAL_MILLISECONDS: 
  		# print('minimum step interval')
  		return #Check that steps arent double counted
  	if currGyroZ < MINIMUM_ACCELERATION_Z:
  		print('minimum acceleration z')
  		return #Check that walker is accelerating forward
  	# xSamples[currSampleCount] = xSampleNew; //Not needed anymore, removal TBI

	if timeDiff >= MINIMUM_STEP_INTERVAL_MILLISECONDS:
		if xSampleNew < xDynamicThreshold:
			lastStepTime = currTime
			numStepsTaken += 1
			totalDist = DIST_PER_STEP_CM * numStepsTaken
			print("Step taken! Total steps - " + str(numStepsTaken) + " ---- AccZ = " + str(currGyroZ))
		else:
			print('greater than dynamic threshold')
			# Serial.println("Step detected but not within interval threshold");
	else:
		print('greater than step interval')

def readAltimu():
	global currGyroZ, currGyroY, xSampleNew
	# Get values from arduino
	xSampleOld = xSampleNew  #Compulsory shift in
	newAccX = int(compass['ax']) - xAccOffset #Get compass from arduino
	diff = abs(newAccX - xSampleOld)
	if diff >= PREDEFINED_PRECISION: #delta is significant enough to shift in
		# xSampleNew = (xFilter[0] + xFilter[1] + xFilter[2] + newAccX)/4.0;  #Averaging over past 3 readings
		incrementSampleCount()
	# Serial.println("Gyro " + (String)gyroX + " " + (String)currGyroY + " " + (String)currGyroZ);

def incrementSampleCount():
	if currSampleCount >= NUM_SAMPLE_COUNTS_TO_RECALCULATE_THRESHOLD - 1:
		calculateNewXThreshold() #Set new threshold
		currSampleCount = 0
	else:
		currSampleCount += 1

def calculateNewXThreshold():
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

for x in xrange(1,500):
	y = fib(22)
	xSampleNew = math.sin(x)
	compass['ax'] = math.sin(x)
	compass['ay'] = math.sin(x)
	compass['az'] = math.sin(x)
	read_step_counter()