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
// Left
#define trigPin1 45
#define echoPin1 44
#define motorPin1 47

// Mid
#define trigPin2 49
#define echoPin2 48
#define motorPin2 51

// Right
#define trigPin3 26
#define echoPin3 27
#define motorPin3 28

long duration, distance, RightSensor, BackSensor, FrontSensor, LeftSensor;
int DIST_THRESHOLD_SIDES = 50;
int DIST_THRESHOLD_MID = 50;
int DURATION_TIMEOUT_SENSOR = 3000;

// Step Counter
int xSampleNew, currGyroY, currGyroZ;
int xFilter[4] = { 0 };
int xSamples[50] = { 0 };
int xDynamicThreshold = 0;
int xSampleOld = 0;
int xAccOffset, yAccOffset, zAccOffset;
int currSampleCount = 0;
int xMin = 0, xMax = 0;
unsigned long lastStepTime;
int numStepsTaken = 0;

int NUM_SAMPLE_COUNTS_TO_RECALCULATE_THRESHOLD = 50;
int MINIMUM_ACCELERATION_Z = 1500;
int MINIMUM_STEP_INTERVAL_MILLISECONDS = 800;
int MINIMUM_ACCELERATION_DELTA = 200; //Crossing below threshold not enough - it must be a decent acceleration change
int DIST_PER_STEP_CM = 75;
int PREDIFINED_PRECISION = 550; //minimum delta to shift new value into xSampleNew (xSampleNew)
float PERCENTAGE_BELOW_DYNAMIC_THRESHOLD_TRIGGER = .2;  //xAcc must be significantly below threshold, TBI
//  ==============================================================================

bool proceed = true;

int phase = 0;

//  ============================  Hardware-Firmware Variables ============================
//TODO: Integrate with Hardware

int dir_value = 0;
int accelx_value = 0;
int accely_value = 0;
int accelz_value = 0;
int sc_timestamp_value = 0;

int dist1_value = 0;
int dist2_value = 0;
int dist3_value = 0;
int od_timestamp_value = 0;

//  ============================  Firmware Variables ============================
#define STACK_SIZE 1024
#define MAX_STORAGE_SIZE 100
#define MAX_SENDING_PACKET_SIZE 10

typedef struct stepCountStorageStrt {
	int dir[MAX_STORAGE_SIZE];
	int accelx[MAX_STORAGE_SIZE];
	int accely[MAX_STORAGE_SIZE];
	int accelz[MAX_STORAGE_SIZE];
	int timestamp[MAX_STORAGE_SIZE];
} StepCountStorage;

typedef struct obstDetectionStrt {
	int dist1[MAX_STORAGE_SIZE];; // Distance from Sensor 1 (Left Arm)
	int dist2[MAX_STORAGE_SIZE];; // Distance from Sensor 2 (Wand)
	int dist3[MAX_STORAGE_SIZE];; // Distance from Sensor 3 (Right Arm)
	int timestamp[MAX_STORAGE_SIZE];;
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
	if (!proceed) return;
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

void readCompass() {
	compass.read();
	float heading = compass.heading((LSM303::vector<int>) {
		0, 0, 1
	});
	Serial.print("readCompass"); Serial.print(" - ");
	Serial.println(heading);
}


void readSensor(int i) {
	Serial.print("readSensor "); Serial.print(i); Serial.print(" - ");
	switch (i) {
		case 0:
			SonarSensor(trigPin1, echoPin1);
			LeftSensor = distance;
			digitalWrite(motorPin1, (LeftSensor <= DIST_THRESHOLD_SIDES) ? HIGH : LOW);
			Serial.print(LeftSensor); Serial.print(" - ");
			break;
		case 1:
			SonarSensor(trigPin2, echoPin2);
			FrontSensor = distance;
			digitalWrite(motorPin2, (FrontSensor <= DIST_THRESHOLD_MID) ? HIGH : LOW);
			Serial.print(FrontSensor); Serial.print(" - ");
			break;
		case 2:
			SonarSensor(trigPin3, echoPin3);
			RightSensor = distance;
			digitalWrite(motorPin3, (RightSensor <= DIST_THRESHOLD_SIDES) ? HIGH : LOW);
			Serial.println(RightSensor);
			break;
	}
}

// ================= Accelerometer

void calculateNewXThreshold() {
	Serial.println("Begin: calculateNewXThreshold");
	xDynamicThreshold = (xMax + xMin) / 2;
	Serial.print("xDynamicThreshold = "); Serial.println(xDynamicThreshold);
	xMax = -9999;
	xMin = 9999;
	Serial.println("End: calculateNewXThreshold");
}

void incrementSampleCount() {
	Serial.println("Begin: incrementSampleCount");
	if (currSampleCount >= NUM_SAMPLE_COUNTS_TO_RECALCULATE_THRESHOLD - 1) {
		calculateNewXThreshold(); //Set new threshold
		currSampleCount = 0;
	}
	else currSampleCount++;
	Serial.println("End: incrementSampleCount");
}

void readAltimu() {
	compass.read();
	xSampleOld = xSampleNew;  //Compulsory shift in
	int newAccX = (int) compass.a.x - xAccOffset;
	int diff = abs(newAccX - xSampleOld);
	if (diff >= PREDIFINED_PRECISION) { //delta is significant enough to shift in
//	    xSampleNew = (xFilter[0] + xFilter[1] + xFilter[2] + newAccX)/4.0;  //Averaging over past 3 readings
		xSampleNew = newAccX; //Shift new value into xSampleNew
		xFilter[3] = xFilter[2];
		xFilter[2] = xFilter[1];
		xFilter[1] = xFilter[0];
		xFilter[0] = xSampleNew;
		if (newAccX > xMax) xMax = newAccX;
		if (newAccX < xMin) xMin = newAccX;
		incrementSampleCount();
	}
	currGyroY = (int) compass.a.y - yAccOffset;
	currGyroZ = (int) compass.a.z - zAccOffset;
	Serial.println("Gyro " + (String)newAccX + " " + (String)currGyroY + " " + (String)currGyroZ);
}

// Excluding function since software will be handling step counting
// TODO: Shift function to software code base
void readStepCounter() {
	Serial.println("Begin: readStepCounter");
	int prevSample = xSampleNew; //Get previous reading
	readAltimu();
	int xAccDelta = abs(prevSample - xSampleNew);
	unsigned long currTime = millis();
	unsigned long timeDiff = currTime - lastStepTime;

	//if (xAccDelta <= MINIMUM_ACCELERATION_DELTA) return;
	if (xAccDelta < MINIMUM_ACCELERATION_DELTA) return;  //Check that walker has accelerated significantly
	if (timeDiff < MINIMUM_STEP_INTERVAL_MILLISECONDS) return; //Check that steps arent double counted
	if (currGyroZ < MINIMUM_ACCELERATION_Z) return; //Check that walker is accelerating forward
	//xSamples[currSampleCount] = xSampleNew; //Not needed anymore, removal TBI

	if (timeDiff >= MINIMUM_STEP_INTERVAL_MILLISECONDS) {
		if (xSampleNew < xDynamicThreshold) {
			lastStepTime = currTime;
			numStepsTaken++;
			int totalDist = DIST_PER_STEP_CM * numStepsTaken;
			Serial.print("Step taken! Total steps - " + (String)numStepsTaken + " ---- AccZ = ");
			Serial.print(currGyroZ);
			Serial.println(" ");
		}
	} else {
		Serial.println("Step detected but not within interval threshold");
	}
}

void calibrate() {
	int NUM_SAMPLES = 128;
	long xGyroSum = 0;
	long yGyroSum = 0;
	long zGyroSum = 0;
	for (int i = 0; i < NUM_SAMPLES; i++) {
		compass.read();
		xGyroSum += compass.a.x;
		yGyroSum += compass.a.y;
		zGyroSum += compass.a.z;
	}
	xAccOffset = xGyroSum / NUM_SAMPLES;
	yAccOffset = yGyroSum / NUM_SAMPLES;
	zAccOffset = zGyroSum / NUM_SAMPLES;
	Serial.println("Calibrated " + (String)xAccOffset + " "
				 + (String)yAccOffset + " " +  (String)zAccOffset);
	xSampleNew = 0;
}

// ================= Hardware Tasks

void getAccelReadings(void *p){
	for (;;) {
		xSemaphoreTake(xSemaphore, portMAX_DELAY);

		readAltimu();

		xSemaphoreGive(xSemaphore);
		vTaskDelay(20);
	}
}

void getCompassReadings(void *p){
	for (;;) {
		xSemaphoreTake(xSemaphore, portMAX_DELAY);

		readCompass();

		xSemaphoreGive(xSemaphore);
		vTaskDelay(20);
	}
}

void getSensorReadings(void *p) {
	for (;;) {
		xSemaphoreTake(xSemaphore, portMAX_DELAY);

		readSensor(0);
		readSensor(1);
		readSensor(2);

		xSemaphoreGive(xSemaphore);
		vTaskDelay(20);
	}
}

//void getSensor1Readings(void *p){
//	for (;;) {
//		xSemaphoreTake(xSemaphore, portMAX_DELAY);
//		readSensor(0);
//		xSemaphoreGive(xSemaphore);
//		vTaskDelay(20);
//	}
//}
//
//void getSensor2Readings(void *p){
//	for (;;) {
//		xSemaphoreTake(xSemaphore, portMAX_DELAY);
//		readSensor(1);
//		xSemaphoreGive(xSemaphore);
//		vTaskDelay(20);
//	}
//}
//
//void getSensor3Readings(void *p){
//	for (;;) {
//		xSemaphoreTake(xSemaphore, portMAX_DELAY);
//		readSensor(2);
//		xSemaphoreGive(xSemaphore);
//		vTaskDelay(20);
//	}
//}

//  ===============================  Firmware Functions  ===============================
static void initialize() {
	bool initFlag = true;

	while (initFlag) {
		if (Serial1.available()) {
			int msg = Serial1.read();
			Serial.println((char) msg);
			//Serial.println("1");
			if (msg == 'h') {
				msg = 'a';
				Serial1.write(msg);

				Serial.println((char) msg);
				if (msg == 'a') {
					initFlag = false;
					//Serial.println("2");
				} else{
					Serial.print("error 3: ");
					Serial.println(msg);
				}
			}
		}
	}
}

// Firmware Testing
// TODO: Integrate with Hardware
void addStepCountDataDebug(void *p) {
	for (;;) {
		xSemaphoreTake(xSemaphore, portMAX_DELAY);

		Serial.println("addStepCountData");

		stepCountStorage.dir[sc_pos_packet] = dir_value;
		stepCountStorage.accelx[sc_pos_packet] = accelx_value;
		stepCountStorage.accely[sc_pos_packet] = accely_value;
		stepCountStorage.accelz[sc_pos_packet] = accelz_value;
		stepCountStorage.timestamp[sc_pos_packet] = sc_timestamp_value;

		sc_pos_packet = (sc_pos_packet + 1) % MAX_STORAGE_SIZE;

		xSemaphoreGive(xSemaphore);
		vTaskDelay(20);
	}
}

// Hardware API: Put Data into Firmware Storage
// TODO: Integrate with Hardware
void addStepCountData(int dir, int accelx, int accely, int accelz,
		int timestamp) {
	Serial.println("addStepCountData");

	stepCountStorage.dir[sc_pos_packet] = dir;
	stepCountStorage.accelx[sc_pos_packet] = accelx;
	stepCountStorage.accely[sc_pos_packet] = accely;
	stepCountStorage.accelz[sc_pos_packet] = accelz;
	stepCountStorage.timestamp[sc_pos_packet] = timestamp;

	sc_pos_packet = (sc_pos_packet + 1) % MAX_STORAGE_SIZE;
}

// TODO: Integrate with Hardware
void transmitStepCountData(void *p) {
	int i = 0;
	for (;;) {
		xSemaphoreTake(xSemaphore, portMAX_DELAY);
		int sc_pos_backup = sc_pos_json;

		Serial.println("transmitStepCountData");

		StaticJsonBuffer<512> jsonBuffer;
		JsonObject& json = jsonBuffer.createObject();
		JsonArray& dir = json.createNestedArray("dir");
		JsonArray& accelx = json.createNestedArray("accelx");
		JsonArray& accely = json.createNestedArray("accely");
		JsonArray& accelz = json.createNestedArray("accelz");
		JsonArray& timestamp = json.createNestedArray("timestamp");
		int checksum = 0;

		for (i = 0; (sc_pos_json != sc_pos_packet) && (i < MAX_SENDING_PACKET_SIZE); i++) {
			dir.add(stepCountStorage.dir[sc_pos_json]);
			accelx.add(stepCountStorage.accelx[sc_pos_json]);
			accely.add(stepCountStorage.accely[sc_pos_json]);
			accelz.add(stepCountStorage.accelz[sc_pos_json]);
			timestamp.add(stepCountStorage.timestamp[sc_pos_json]);
			checksum = (checksum + stepCountStorage.timestamp[sc_pos_json]) %256;
			sc_pos_json = (sc_pos_json + 1) % MAX_STORAGE_SIZE;
		}

		json["checksum"] = checksum;

		//might fuck up
		while(!Serial1){

		}

		json.printTo(Serial1);

		//might fuck up
		while (!(Serial1 && Serial1.available())){

		}

		if (Serial1.available()){
			char msg = Serial1.read();
			if (msg =='n'){
				sc_pos_json = sc_pos_backup;
			}
			Serial.println(msg);
		}
		else {
			Serial.println("error");
			sc_pos_json = sc_pos_backup;
		}

		xSemaphoreGive(xSemaphore);
		vTaskDelay(100);
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
	pinMode(motorPin1, OUTPUT);
	pinMode(motorPin2, OUTPUT);
	pinMode(motorPin3, OUTPUT);


	// Compass
	Wire.begin();
	delayMicroseconds(10);

	Serial.println("Starting compass!");
	if (!compass.init())
	{
		Serial.println("Failed to initialize compass!");
		while (1);
	} else {
		Serial.println("Compass Initialized!");
	}

	compass.enableDefault();
	compass.m_min = (LSM303::vector<int16_t>) {
		-32767, -32767, -32767
	};
	compass.m_max = (LSM303::vector<int16_t>) {
		+32767, +32767, +32767
	};

	calibrate();

	//  ===============================  Create Hardware Tasks  ===============================
//	xTaskCreate(getAccelReadings, "getAccelReadings", 6*STACK_SIZE, NULL, 1, NULL); // NOT WORKING; TODO: FIX THIS
//	xTaskCreate(getCompassReadings, "getCompassReadings", 4*STACK_SIZE, NULL, 1, NULL);
	xTaskCreate(getSensorReadings, "getSensorReadings", 6*STACK_SIZE, NULL, 2, NULL);
//	xTaskCreate(getSensor1Readings, "getSensor1Readings", 4*STACK_SIZE, NULL, 2, NULL); // Use only if getSensorReadings don't work
//	xTaskCreate(getSensor2Readings, "getSensor2Readings", 4*STACK_SIZE, NULL, 2, NULL); // Use only if getSensorReadings don't work
//	xTaskCreate(getSensor3Readings, "getSensor3Readings", 4*STACK_SIZE, NULL, 2, NULL); // Use only if getSensorReadings don't work

	//  ===============================  Setup Firmware Connection  ===============================
	Serial.flush();
	// Serial1.flush();

	// Serial.println("Begin Arduino-Pi Connection");
	// initialize();
	// Serial.println("Connection Established. Sending data from Arduino to Pi.");

	//  ===============================  Create Firmware Tasks  ===============================
	// xTaskCreate(addStepCountDataDebug, "addStepCountData", STACK_SIZE, NULL, 1, NULL);
	// xTaskCreate(transmitStepCountData, "transmitStepCountData", STACK_SIZE, NULL, 1, NULL);
}

void loop() {

}
