#include <ArduinoJson.h>
#include <Arduino_FreeRTOS.h>
#include <SoftwareSerial.h>

#include <Arduino.h>
#include <avr/io.h>
#include <task.h>
#include <semphr.h>

#include <Wire.h>
#include <LSM303.h>
#include <L3G.h>
LSM303 compass;
L3G gyro;

//  ============================  Hardware Variables ============================
// Ultrasound Sensors
// Left Arm
#define trigPin1 26
#define echoPin1 27
#define motorPin1 28

// Mid Arm
#define trigPin2 49
#define echoPin2 48
#define motorPin2 51

// Right Arm
#define trigPin3 32
#define echoPin3 34
#define motorPin3 35

//Left Leg
#define trigPin4 40
#define echoPin4 41
#define motorPin4 43

//Right Leg
#define trigPin5 45
#define echoPin5 44
#define motorPin5 47

long duration, distance, rightArmSensor, frontSensor, leftArmSensor,
		leftLegSensor, rightLegSensor;
int DIST_THRESHOLD_SIDES = 50;
int DIST_THRESHOLD_MID = 50;
int DURATION_TIMEOUT_SENSOR = 3000;
int totalDist = 0;

// Step Counter
volatile int xSampleNew, currAccY, currAccZ;
volatile int xAccHistory[4] = { 0 };
volatile int zAccHistory[4] = { 0 };
volatile int xSamples[50] = { 0 };
volatile int xDynamicThreshold = 0;
volatile int xSampleOld = 0;
int xAccOffset, yAccOffset, zAccOffset;
volatile int currSampleCount = 0;
volatile int xMin = 0, xMax = 0;
unsigned long lastStepTime;
int numStepsTaken = 0;
volatile long lastKnownMagnitude = 0;

volatile int lastKnownDirection;
volatile float lastDistanceTaken;

int NUM_SAMPLE_COUNTS_TO_RECALCULATE_THRESHOLD = 50;
int MINIMUM_ACCELERATION_MAGNITUDE = 8500;
int MINIMUM_STEP_INTERVAL_MILLISECONDS = 800;
int MINIMUM_ACCELERATION_DELTA = 200; //Crossing below threshold not enough - it must be a decent acceleration change
int DIST_PER_STEP_CM = 75;
int PREDIFINED_PRECISION = 250; //minimum delta to shift new value into xSampleNew (xSampleNew)
float PERCENTAGE_BELOW_DYNAMIC_THRESHOLD_TRIGGER = .2; //xAcc must be significantly below threshold, TBI
//  ==============================================================================

bool proceed = true;

int phase = 0;

//  ============================  Hardware-Firmware Variables ============================
//TODO: Integrate with Hardware

int dir_value = 0;
int accelx_value = 0;
int accely_value = 0;
int accelz_value = 0;
long sc_timestamp_value = 0;

int dist1_value = 0;
int dist2_value = 0;
int dist3_value = 0;
long od_timestamp_value = 0;

//  ============================  Firmware Variables ============================
#define STACK_SIZE 1024
#define MAX_STORAGE_SIZE 100
#define MAX_SENDING_PACKET_SIZE 10

boolean isSendingData = false;

typedef struct stepCountStorageStrt {
	volatile int direction;
	volatile float distance;
} StepCountStorage;

typedef struct backupStepCountStorageStrt {
	volatile int direction;
	volatile float distance;
} BackupStepCountStorage;

typedef struct obstDetectionStrt {
	int dist1[MAX_STORAGE_SIZE]; // Distance from Sensor 1 (Left Arm)
	int dist2[MAX_STORAGE_SIZE]; // Distance from Sensor 2 (Wand)
	int dist3[MAX_STORAGE_SIZE]; // Distance from Sensor 3 (Right Arm)
	long timestamp[MAX_STORAGE_SIZE];
} ObstDetectionStorage;

StepCountStorage stepCountStorage;
ObstDetectionStorage obstDetectionStorage;

int sc_pos_packet = 0;
int sc_pos_json = 0;

int od_pos_packet = 0;
int od_pos_json = 0;

SemaphoreHandle_t xSemaphore = NULL;

QueueHandle_t xQueue = NULL;

//  ===============================  Hardware Functions  ===============================

void SonarSensor(int trigPin, int echoPin) {
	if (!proceed)
		return;
	proceed = false;
	distance = 9999;
	digitalWrite(trigPin, LOW);
	delayMicroseconds(2);
	digitalWrite(trigPin, HIGH);
	delayMicroseconds(10);
	digitalWrite(trigPin, LOW);
	duration = pulseIn(echoPin, HIGH, DURATION_TIMEOUT_SENSOR);
	if (duration == 0)
		duration = DURATION_TIMEOUT_SENSOR;
	distance = (duration / 2) / 29.1;
	proceed = true;
}

void readSensor(int i) {
	Serial.print("Sensor ");
	Serial.print(i);
	Serial.print(" ");
	switch (i) {
	case 0:
		SonarSensor(trigPin1, echoPin1);
		leftArmSensor = distance;
		digitalWrite(motorPin1,
				(leftArmSensor <= DIST_THRESHOLD_SIDES) ? HIGH : LOW);
		Serial.println(leftArmSensor);
		break;
	case 1:
		SonarSensor(trigPin2, echoPin2);
		frontSensor = distance;
		digitalWrite(motorPin2,
				(frontSensor <= DIST_THRESHOLD_MID) ? HIGH : LOW);
		Serial.println(frontSensor);
		break;
	case 2:
		SonarSensor(trigPin3, echoPin3);
		rightArmSensor = distance;
		digitalWrite(motorPin3,
				(rightArmSensor <= DIST_THRESHOLD_SIDES) ? HIGH : LOW);
		Serial.println(rightArmSensor);
		break;
	case 3:
		SonarSensor(trigPin4, echoPin4);
		leftLegSensor = distance;
		digitalWrite(motorPin4,
				(leftLegSensor <= DIST_THRESHOLD_SIDES) ? HIGH : LOW);
		Serial.println(leftLegSensor);
		break;
	case 4:
		SonarSensor(trigPin5, echoPin5);
		rightLegSensor = distance;
		digitalWrite(motorPin5,
				(rightLegSensor <= DIST_THRESHOLD_SIDES) ? HIGH : LOW);
		Serial.println(rightLegSensor);
		break;
	}
}

// ================= Accelerometer

//// Hardware-Firmware API: Put Data into Firmware Storage
//void addStepCountData(int dir, int accelx, int accely, int accelz,
//		long timestamp) {
//	Serial.println("addStepCountData");
//
//	stepCountStorage.dir[sc_pos_packet] = dir;
//	stepCountStorage.accelx[sc_pos_packet] = accelx;
//	stepCountStorage.accely[sc_pos_packet] = accely;
//	stepCountStorage.accelz[sc_pos_packet] = accelz;
//	stepCountStorage.timestamp[sc_pos_packet] = timestamp;
//
//	sc_pos_packet = (sc_pos_packet + 1) % MAX_STORAGE_SIZE;
//}

void calculateNewXThreshold() {
	xDynamicThreshold = (xMax + xMin) / 2;
	Serial.print("xDynamicThreshold = ");
	Serial.println(xDynamicThreshold);
	xMax = -9999;
	xMin = 9999;
}

void incrementSampleCount() {
	if (currSampleCount >= NUM_SAMPLE_COUNTS_TO_RECALCULATE_THRESHOLD - 1) {
		calculateNewXThreshold(); //Set new threshold
		currSampleCount = 0;
	} else
		currSampleCount++;
}

void readAltimu() {
	compass.read();
	xSampleOld = xSampleNew;  //Compulsory shift in

	int newAccX = (int) compass.a.x;
	int newAccY = (int) compass.a.y;
	int newAccZ = (int) compass.a.z;

	long xSquare = (long) newAccX * (long) newAccX;
	long ySquare = (long) newAccY * (long) newAccY;
	long zSquare = (long) newAccZ * (long) newAccZ;

	newAccX -= xAccOffset;
	newAccY -= yAccOffset;
	newAccZ -= zAccOffset;

	int accDueToGravity = 9810;
//	Serial.print("X "); Serial.print(newAccX); Serial.print(" --- Y "); Serial.print(newAccY); Serial.print(" --- Z "); Serial.println(newAccZ);
	lastKnownMagnitude = (long) sqrt((xSquare) + (ySquare) + (zSquare))
			- accDueToGravity;
//	Serial.print("LastKnownMag "); Serial.println(lastKnownMagnitude);

	int xAccDiff = abs(newAccX - xSampleOld);
	if (xAccDiff >= PREDIFINED_PRECISION) { //delta is significant enough to shift in
//	    xSampleNew = (xAccHistory[0] + xAccHistory[1] + xAccHistory[2] + newAccX)/4.0;  //Averaging over past 3 readings
		xSampleNew = newAccX; //Shift new value into xSampleNew
		xAccHistory[3] = xAccHistory[2];
		xAccHistory[2] = xAccHistory[1];
		xAccHistory[1] = xAccHistory[0];
		xAccHistory[0] = xSampleNew;
		if (newAccX > xMax)
			xMax = newAccX;
		if (newAccX < xMin)
			xMin = newAccX;
		incrementSampleCount();
	}

	currAccY = newAccY;
	currAccZ = newAccZ;

	//Record zAcceleration values
	zAccHistory[3] = zAccHistory[2];
	zAccHistory[2] = zAccHistory[1];
	zAccHistory[1] = zAccHistory[0];
	zAccHistory[0] = currAccZ;

//	Serial.print("AccX "); Serial.println(newAccX);
//	Serial.print("AccY "); Serial.println(currAccY);
//	Serial.print("AccZ "); Serial.println(currAccZ);

	lastKnownDirection = (int) compass.heading(
			(LSM303::vector<int> ) { 0, 0, 1 });
//	long timeMillis = millis();
//	addStepCountData(compassReadings, newAccX, currAccY, currAccZ, timeMillis);
}

void calibrate() {
	int NUM_SAMPLES = 128;
	long xAccSum = 0;
	long yAccSum = 0;
	long zAccSum = 0;
	for (int i = 0; i < NUM_SAMPLES; i++) {
		compass.read();
		xAccSum += compass.a.x;
		yAccSum += compass.a.y;
		zAccSum += compass.a.z;
		delay(10);	//###Lengthen the sampling time
	}
	xAccOffset = xAccSum / NUM_SAMPLES;
	yAccOffset = yAccSum / NUM_SAMPLES;
	zAccOffset = zAccSum / NUM_SAMPLES;
	Serial.println(
			"Calibrated " + (String) xAccOffset + " " + (String) yAccOffset
					+ " " + (String) zAccOffset);
	xSampleNew = 0;
}

// ================= Hardware Tasks

void getAccelReadings(void *p) {
	for (;;) {
		xSemaphoreTake(xSemaphore, portMAX_DELAY);

		int prevSample = xSampleNew; //Get previous reading
		readAltimu();

		//Now check if step is taken
		float distanceTaken = 0;
		int xAccDelta = abs(prevSample - xSampleNew);
		unsigned long currTime = millis();
		unsigned long timeDiff = currTime - lastStepTime;
		//if (xAccDelta <= MINIMUM_ACCELERATION_DELTA) return;
		//if (xAccDelta >= MINIMUM_ACCELERATION_DELTA) {
		//Check that walker has accelerated significantly
		if (timeDiff >= MINIMUM_STEP_INTERVAL_MILLISECONDS) {
			//Check that steps arent double counted
			if (lastKnownMagnitude >= MINIMUM_ACCELERATION_MAGNITUDE) {
				//Check that walker is accelerating forward
				if (timeDiff >= MINIMUM_STEP_INTERVAL_MILLISECONDS) {
					//if (xSampleNew > xDynamicThreshold) {
					lastStepTime = currTime;
					numStepsTaken++;
					totalDist = DIST_PER_STEP_CM * numStepsTaken;
					distanceTaken += DIST_PER_STEP_CM;
					Serial.print("Step taken!");
					Serial.print(numStepsTaken);
					Serial.println(" ");
					//} else {
					//	Serial.println("Dynamic threshold not met");
					//}
				} else {
					//      Serial.println("Step detected but not within interval threshold");
				}
			} else {
				//Serial.print("Magnitude not met --- "); Serial.println(lastKnownMagnitude);
			}
		} else {
			//Serial.println("Step interval timing not met");
		}
		//}

		lastDistanceTaken += distanceTaken;
		xSemaphoreGive(xSemaphore);
		vTaskDelay(1);
	}
}

void getSensor1Readings(void *p) {
	for (;;) {
		xSemaphoreTake(xSemaphore, portMAX_DELAY);

		readSensor(0);

		xSemaphoreGive(xSemaphore);
		vTaskDelay(25);
	}
}

void getSensor2Readings(void *p) {
	for (;;) {
		xSemaphoreTake(xSemaphore, portMAX_DELAY);

		readSensor(1);

		xSemaphoreGive(xSemaphore);
		vTaskDelay(25);
	}
}

void getSensor3Readings(void *p) {
	for (;;) {
		xSemaphoreTake(xSemaphore, portMAX_DELAY);

		readSensor(2);

		xSemaphoreGive(xSemaphore);
		vTaskDelay(25);
	}
}

void getSensorReadings(void *p) {
	for (;;) {
		xSemaphoreTake(xSemaphore, portMAX_DELAY);

		readSensor(0);
		readSensor(1);
		readSensor(2);
		readSensor(3);
		readSensor(4);

		xSemaphoreGive(xSemaphore);
		vTaskDelay(25);
	}
}

//  ===============================  Firmware Functions  ===============================
static void initialize(bool firstSetup) {
	bool handshake = true;

	if (!firstSetup) {
		Serial.flush();
		Serial1.flush();
	}

	while (handshake) {
		int msg;
		if (firstSetup && Serial1.available()) {
			int msg = Serial1.read();
			Serial.println((char) msg);
		}
		if ((!firstSetup) || (msg == 'h')) {
			msg = 'a';
			Serial1.write(msg);
		} else {
			Serial.println("Handshake: error 2");
		}
		if (Serial1.available()) {
			int msg = Serial1.read();
			if (msg == 'a') {
				initFlag = false;
			}
		} else {
			Serial.println("Handshake: error 3");
		}
	}
}

//// Firmware Testing
//// TODO: Integrate with Hardware
//void addStepCountDataDebug(void *p) {
//	for (;;) {
//		xSemaphoreTake(xSemaphore, portMAX_DELAY);
//
//		Serial.println("addStepCountData");
//
//		stepCountStorage.dir[sc_pos_packet] = dir_value;
//		stepCountStorage.accelx[sc_pos_packet] = accelx_value;
//		stepCountStorage.accely[sc_pos_packet] = accely_value;
//		stepCountStorage.accelz[sc_pos_packet] = accelz_value;
//		stepCountStorage.timestamp[sc_pos_packet] = sc_timestamp_value;
//
//		sc_pos_packet = (sc_pos_packet + 1) % MAX_STORAGE_SIZE;
//
//		xSemaphoreGive(xSemaphore);
//		vTaskDelay(1);
//	}
//}

// TODO: Integrate with Hardware
void transmitStepCountData(void *p) {
	int i = 0;
	for (;;) {
		xSemaphoreTake(xSemaphore, portMAX_DELAY);
		isSendingData = true;

		Serial.println("transmitStepCountData");

		StaticJsonBuffer<512> jsonBuffer;
		JsonObject& json = jsonBuffer.createObject();
		json["direction"] = lastKnownDirection;
		json["distance"] = totalDist;
		long checksum = (lastKnownDirection + (int) totalDist) % 256;//### Must make rPi recompute checksum too
		json["checksum"] = checksum;

		/*		JsonArray& dir = json.createNestedArray("dir");
		 JsonArray& accelx = json.createNestedArray("accelx");
		 JsonArray& accely = json.createNestedArray("accely");
		 JsonArray& accelz = json.createNestedArray("accelz");
		 JsonArray& timestamp = json.createNestedArray("timestamp");
		 long checksum = 0;

		 for (i = 0; (sc_pos_json != sc_pos_packet) && (i < MAX_SENDING_PACKET_SIZE); i++) {
		 dir.add(stepCountStorage.dir[sc_pos_json]);
		 accelx.add(stepCountStorage.accelx[sc_pos_json]);
		 accely.add(stepCountStorage.accely[sc_pos_json]);
		 accelz.add(stepCountStorage.accelz[sc_pos_json]);
		 timestamp.add(stepCountStorage.timestamp[sc_pos_json]);
		 checksum = (checksum + stepCountStorage.timestamp[sc_pos_json]) %256;
		 sc_pos_json = (sc_pos_json + 1) % MAX_STORAGE_SIZE;
		 } */

		while (!Serial1) {
		}
		json.printTo(Serial1);
		while (!(Serial1 && Serial1.available())) {
		}

		if (Serial1.available()) {
			char msg = Serial1.read();

			if (msg == 'n') {
				//sc_pos_json = sc_pos_backup;
			}
			if (msg == 'h') {
				initialize(false);
			}

			Serial.println("transmit msg: " + msg);
		} else {
			Serial.println("error");
			//sc_pos_json = sc_pos_backup;
		}

		xSemaphoreGive(xSemaphore);
		vTaskDelay(25);
	}
}

void setup() {
//  ===============================  Initialise  ===============================
	xSemaphore = xSemaphoreCreateBinary();
	xSemaphoreGive(xSemaphore);

	Serial.begin(9600);
	Serial1.begin(9600);

//  ===============================  Setup Hardware Connection  ===============================
	Serial.println("Initializing Hardware!");
	pinMode(trigPin1, OUTPUT);
	pinMode(echoPin1, INPUT);
	pinMode(trigPin2, OUTPUT);
	pinMode(echoPin2, INPUT);
	pinMode(trigPin3, OUTPUT);
	pinMode(echoPin3, INPUT);
	pinMode(trigPin4, OUTPUT);
	pinMode(echoPin4, INPUT);
	pinMode(trigPin5, OUTPUT);
	pinMode(echoPin5, INPUT);

	pinMode(motorPin1, OUTPUT);
	pinMode(motorPin2, OUTPUT);
	pinMode(motorPin3, OUTPUT);
	pinMode(motorPin4, OUTPUT);
	pinMode(motorPin5, OUTPUT);

// Compass
	Wire.begin();
	delayMicroseconds(10);

	Serial.println("Starting compass!");
	if (!compass.init()) {
		Serial.println("Failed to initialize compass!");
		while (1)
			;
	} else {
		Serial.println("Compass Initialized!");
	}

	compass.enableDefault();
	compass.m_min = (LSM303::vector<int16_t> ) { -32767, -32767, -32767 };
	compass.m_max = (LSM303::vector<int16_t> ) { +32767, +32767, +32767 };

	calibrate();

//  ===============================  Create Hardware Tasks  ===============================
	xTaskCreate(getAccelReadings, "getAccelReadings", 3 * STACK_SIZE, NULL, 1,
	NULL);
//	xTaskCreate(getSensorReadings, "getSensorReadings", STACK_SIZE, NULL, 1, NULL);
//	xTaskCreate(getSensor1Readings, "getSensor1Readings", STACK_SIZE, NULL, 2, NULL); // Use only if getSensorReadings is not working
//	xTaskCreate(getSensor2Readings, "getSensor2Readings", STACK_SIZE, NULL, 2, NULL); // Use only if getSensorReadings is not working
//	xTaskCreate(getSensor3Readings, "getSensor3Readings", STACK_SIZE, NULL, 2, NULL); // Use only if getSensorReadings is not working

//  ===============================  Setup Firmware Connection  ===============================
	Serial.flush();
//	Serial1.flush();

//	Serial.println("Begin Arduino-Pi Connection");
//	initialize(true);
//	Serial.println("Connection Established. Sending data from Arduino to Pi.");

//  ===============================  Create Firmware Tasks  ===============================
//	xTaskCreate(addStepCountDataDebug, "addStepCountData", STACK_SIZE, NULL, 1, NULL); // Debugging Purpose
//	xTaskCreate(transmitStepCountData, "transmitStepCountData", STACK_SIZE, NULL, 2, NULL); // Keep Transmit at Lower Priority than Sensor Readings

	vTaskStartScheduler();
}

void loop() {

}
