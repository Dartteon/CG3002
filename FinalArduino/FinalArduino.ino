#include <ArduinoJson.h>
#include <Arduino_FreeRTOS.h>
#include <SoftwareSerial.h>
#include <NewPing.h>

#include <Arduino.h>
#include <avr/io.h>
#include <task.h>
#include <semphr.h>

#include <Wire.h>
#include <LSM303.h>
#include <L3G.h>
LSM303 compass;
L3G gyro;

bool DEBUG_ARDUINO_ONLY = false;

//  ============================  Hardware Variables ============================
// Ultrasound Sensors
// Left Arm
#define trigPin1 26
#define echoPin1 27
#define motorPin1 28

// Mid Arm
#define trigPin2 22
#define echoPin2 23
#define motorPin2 24
//#define trigPin2 49
//#define echoPin2 48
//#define motorPin2 51

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

long rightArmSensor, frontSensor, leftArmSensor,
     leftLegSensor, rightLegSensor;
int DIST_THRESHOLD_SIDES = 100;
int DIST_THRESHOLD_MID = 100;
int DURATION_TIMEOUT_SENSOR = 3000;
float TIMEOUT_MULTIPLIER = 1.96; //50cm for each multiple

//NewPing Sensors
#define SONAR_NUM     5 // Number or sensors.
#define MAX_DISTANCE 100 // Max distance in cm.
#define MIN_DISTANCE 5 // Max distance in cm.
#define PING_INTERVAL 33 // Milliseconds between pings.
unsigned long pingTimer[SONAR_NUM]; // When each pings.
unsigned int cm[SONAR_NUM]; // Store ping distances.
uint8_t currentSensor = 0; // Which sensor is active.
NewPing sonar[SONAR_NUM] = { // Sensor object array.
  NewPing(trigPin1, echoPin1, MAX_DISTANCE),
  NewPing(trigPin2, echoPin2, MAX_DISTANCE),
  NewPing(trigPin3, echoPin3, MAX_DISTANCE),
  NewPing(trigPin4, echoPin4, MAX_DISTANCE),
  NewPing(trigPin5, echoPin5, MAX_DISTANCE)
};
int motorPins[SONAR_NUM];

// Step Counter
volatile int xSampleNew, currAccY, currAccZ;
volatile int xAccHistory[4] = { 0 };
volatile int zAccHistory[4] = { 0 };
volatile int xSamples[50] = { 0 };
volatile int xDynamicThreshold = 0;
volatile int xSampleOld = 0;
int xAccOffset, yAccOffset, zAccOffset;
long magnitudeAccOffset;
volatile int currSampleCount = 0;
volatile int xMin = 0, xMax = 0;
unsigned long lastStepTime;
int numStepsTaken = 0;
volatile long lastKnownMagnitude = 0;

volatile int lastKnownDirection;

int NUM_SAMPLE_COUNTS_TO_RECALCULATE_THRESHOLD = 50;
int MINIMUM_ACCELERATION_MAGNITUDE = 5000;
int MINIMUM_STEP_INTERVAL_MILLISECONDS = 800;
int MINIMUM_ACCELERATION_DELTA = 200; //Crossing below threshold not enough - it must be a decent acceleration change
int DIST_PER_STEP_CM = 60;
int PREDIFINED_PRECISION = 250; //minimum delta to shift new value into xSampleNew (xSampleNew)

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

SemaphoreHandle_t xSemaphore = NULL;

QueueHandle_t xQueue = NULL;

//  ===============================  Hardware Functions  ===============================

void SonarSensor(int trigPin, int echoPin) {
//  distance = 9999;
//  digitalWrite(trigPin, LOW);
//  delayMicroseconds(2);
//  digitalWrite(trigPin, HIGH);
//  delayMicroseconds(10);
//  digitalWrite(trigPin, LOW);
//  int timeOut = DURATION_TIMEOUT_SENSOR;
//  timeOut *= TIMEOUT_MULTIPLIER;
//  //if (trigPin == trigPin2 || trigPin == trigPin4 || trigPin == trigPin5) timeOut *= 2;
//  duration = pulseIn(echoPin, HIGH, timeOut);
//  if (duration == 0)
//    duration = timeOut;
//  distance = (duration / 2) / 29.1;
}

//void readSensor(int i) {
//  int trigPin;
//  int echoPin;
//  int distance1 = 9999;
//  int duration1 = 9999;
//  int distance2 = 9999;
//  int duration2 = 9999;
//  int distance3 = 9999;
//  int duration3 = 9999;
//  int distance4 = 9999;
//  int duration4 = 9999;
//  int distance5 = 9999;
//  int duration5 = 9999;
//  int timeOut = DURATION_TIMEOUT_SENSOR;
//  timeOut *= TIMEOUT_MULTIPLIER;
//  switch (i) {
//    case 0:
//      trigPin = trigPin1;
//      echoPin = echoPin1;
//      //============== SonarSensor
//      digitalWrite(trigPin, LOW);
//      delayMicroseconds(2);
//      digitalWrite(trigPin, HIGH);
//      delayMicroseconds(10);
//      digitalWrite(trigPin, LOW);
//      duration1 = pulseIn(echoPin, HIGH, timeOut);
//      if (duration1 == 0) duration1 = timeOut;
//      distance1 = (duration1 / 2) / 29.1;
//      //============== EndSonarSensor
//      leftArmSensor = (duration1 / 2) / 29.1;
//      digitalWrite(motorPin1, (leftArmSensor <= DIST_THRESHOLD_SIDES) ? HIGH : LOW);
//      Serial.print("Left Arm - ");
//      Serial.println(leftArmSensor);
//      break;
//    case 1:
//      trigPin = trigPin2;
//      echoPin = echoPin2;
//      //============== SonarSensor
//      digitalWrite(trigPin, LOW);
//      delayMicroseconds(2);
//      digitalWrite(trigPin, HIGH);
//      delayMicroseconds(10);
//      digitalWrite(trigPin, LOW);
//      duration2 = pulseIn(echoPin, HIGH, timeOut);
//      if (duration2 == 0) duration2 = timeOut;
//      distance2 = (duration2 / 2) / 29.1;
//      //============== EndSonarSensor
//      frontSensor = (duration2 / 2) / 29.1;
//      digitalWrite(motorPin2, (frontSensor <= DIST_THRESHOLD_MID) ? HIGH : LOW);
//      Serial.print("Front - ");
//      Serial.println(frontSensor);
//      break;
//    case 2:
//      trigPin = trigPin3;
//      echoPin = echoPin3;
//      //============== SonarSensor
//      digitalWrite(trigPin, LOW);
//      delayMicroseconds(2);
//      digitalWrite(trigPin, HIGH);
//      delayMicroseconds(10);
//      digitalWrite(trigPin, LOW);
//      duration3 = pulseIn(echoPin, HIGH, timeOut);
//      if (duration3 == 0) duration3 = timeOut;
//      distance3 = (duration3 / 2) / 29.1;
//      //============== EndSonarSensor
//      rightArmSensor = (duration3 / 2) / 29.1;
//      digitalWrite(motorPin3, (rightArmSensor <= DIST_THRESHOLD_SIDES) ? HIGH : LOW);
//      Serial.print("Right Arm - ");
//      Serial.println(rightArmSensor);
//      break;
//    case 3:
//      trigPin = trigPin4;
//      echoPin = echoPin4;
//      //============== SonarSensor
//      digitalWrite(trigPin, LOW);
//      delayMicroseconds(2);
//      digitalWrite(trigPin, HIGH);
//      delayMicroseconds(10);
//      digitalWrite(trigPin, LOW);
//      duration4 = pulseIn(echoPin, HIGH, timeOut);
//      if (duration4 == 0) duration4 = timeOut;
//      distance4 = (duration4 / 2) / 29.1;
//      //============== EndSonarSensor
//      leftLegSensor = (duration4 / 2) / 29.1;
//      digitalWrite(motorPin4, (leftLegSensor <= DIST_THRESHOLD_SIDES) ? HIGH : LOW);
//      Serial.print("Left Leg - ");
//      Serial.println(leftLegSensor);
//      break;
//    case 4:
//      trigPin = trigPin5;
//      echoPin = echoPin5;
//      //============== SonarSensor
//      digitalWrite(trigPin, LOW);
//      delayMicroseconds(2);
//      digitalWrite(trigPin, HIGH);
//      delayMicroseconds(10);
//      digitalWrite(trigPin, LOW);
//      duration5 = pulseIn(echoPin, HIGH, timeOut);
//      if (duration5 == 0) duration5 = timeOut;
//      distance5 = (duration5 / 2) / 29.1;
//      //============== EndSonarSensor
//      rightLegSensor = (duration5 / 2) / 29.1;
//      digitalWrite(motorPin5, (rightLegSensor <= DIST_THRESHOLD_SIDES) ? HIGH : LOW);
//      Serial.print("Right Leg - ");
//      Serial.println(rightLegSensor);
//      break;
//  }
//}
void echoCheck() { // If ping echo, set distance to array.
  if (sonar[currentSensor].check_timer())
    cm[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;
}
void oneSensorCycle() { // Do something with the results.
  for (uint8_t i = 0; i < SONAR_NUM; i++) {
    unsigned int currDist = cm[i];
    digitalWrite(motorPins[i], (currDist > MIN_DISTANCE && currDist <= MAX_DISTANCE));
    Serial.print(cm[i]);
    Serial.print(" ");
  }
  Serial.println();
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
  //	Serial.print("xDynamicThreshold = ");
  //	Serial.println(xDynamicThreshold);
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

  //	int accDueToGravity = 9810;
  //	Serial.print("X "); Serial.print(newAccX); Serial.print(" --- Y "); Serial.print(newAccY); Serial.print(" --- Z "); Serial.println(newAccZ);
  lastKnownMagnitude = (long) sqrt((xSquare) + (ySquare) + (zSquare))
                       - magnitudeAccOffset;
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
  (LSM303::vector<int> ) {
    0, 0, 1
  });
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


  long xSquare = (long) xAccOffset * (long) xAccOffset;
  long ySquare = (long) yAccOffset * (long) yAccOffset;
  long zSquare = (long) zAccOffset * (long) zAccOffset;

  magnitudeAccOffset = sqrt(xSquare + ySquare + zSquare);
  Serial.println(
    "Calibrated " + (String) xAccOffset + " " + (String) yAccOffset
    + " " + (String) zAccOffset);
  Serial.println("MagOffset " + (String)magnitudeAccOffset);

  xSampleNew = 0;
}

// ================= Hardware Tasks

void getAccelReadings(void *p) {
  for (;;) {
    xSemaphoreTake(xSemaphore, portMAX_DELAY);

    //Retrieve data from altimu
    readAltimu();

    //Now check if step is taken
    unsigned long currTime = millis();
    unsigned long timeDiff = currTime - lastStepTime;

    //Check that walker has accelerated significantly
    if (timeDiff >= MINIMUM_STEP_INTERVAL_MILLISECONDS) {
      //Check that steps arent double counted
      if (lastKnownMagnitude >= MINIMUM_ACCELERATION_MAGNITUDE) {
        //Check that walker is accelerating forward
        if (timeDiff >= MINIMUM_STEP_INTERVAL_MILLISECONDS) {
          //if (xSampleNew > xDynamicThreshold) {
          lastStepTime = currTime;
          numStepsTaken++;
          Serial.print("Step taken! --- ");
          Serial.print(numStepsTaken);
          Serial.println(" ");
          //} else {
          //	Serial.println("Dynamic threshold not met");
          //}
        } else {
          //			        Serial.println("Step detected but not within interval threshold");
        }
      } else {
        //				Serial.print("Magnitude not met --- "); Serial.println(lastKnownMagnitude);
      }
    } else {
      //			Serial.println("Step interval timing not met");
    }
    //}

    xSemaphoreGive(xSemaphore);
    vTaskDelay(5);
  }
}

void getSensorReadings(void *p) {
  int sensorCounter = 0;
  for (;;) {
    xSemaphoreTake(xSemaphore, portMAX_DELAY);
//    readSensor(sensorCounter);
//    sensorCounter = (sensorCounter + 1) % 5;

    //NewPing Sensors
    for (uint8_t i = 0; i < SONAR_NUM; i++) {
      if (millis() >= pingTimer[i]) {
        pingTimer[i] += PING_INTERVAL * SONAR_NUM;
        if (i == 0 && currentSensor == SONAR_NUM - 1)
          oneSensorCycle(); // Do something with results.
        sonar[currentSensor].timer_stop();
        currentSensor = i;
        cm[currentSensor] = 0;
        sonar[currentSensor].ping_timer(echoCheck);
      }
    }
    
    xSemaphoreGive(xSemaphore);
    vTaskDelay(1);
  }
}

static void serialFlush() {
  while (Serial.available() > 0) {
    char t = Serial.read();
  }
  while (Serial1.available() > 0) {
    char t = Serial.read();
  }
}

//  ===============================  Firmware Functions  ===============================
static void initialize() {
  bool initFlag = true;
  serialFlush();
  while (initFlag) {

    Serial.println("Expecting 'h'");
    while (!Serial1.available()) {
    }
    int msg = Serial1.read();
    Serial.println((char) msg);
    if (msg == 'h') {
      msg = 'a';
      Serial1.write(msg);
    }
    else {
      Serial.println("Unexpected handshake protocol sequence, forced bypass");
      msg = 'a';
      Serial1.write(msg);
    }

    Serial.println("Expecting 'a'");
    while (!Serial1.available()) {
    }
    msg = Serial1.read();
    Serial.println((char) msg);
    if (msg == 'a') {
      initFlag = false;
    } else {
      Serial.println("Unexpected handshake protocol sequence, forced bypass");
      initFlag = false;
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
    //xSemaphoreTake(xSemaphore, portMAX_DELAY);
    isSendingData = true;

    StaticJsonBuffer<256> jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    json["direction"] = lastKnownDirection;
    json["distance"] = numStepsTaken;
    long checksum = (lastKnownDirection + numStepsTaken) % 256;//### Must make rPi recompute checksum too
    json["checksum"] = checksum;

    Serial.print("numStepsTaken: ");
    Serial.println(numStepsTaken);

    while (!Serial1) {
    }

    json.printTo(Serial1);
    Serial1.println("");
    json.printTo(Serial);
    Serial.println("");

    while (!(Serial1 && Serial1.available())) {
    }
    if (Serial1.available()) {
      char msg = Serial1.read();
      Serial.println(msg);
      if (msg == 'n') {
        //sc_pos_json = sc_pos_backup;
      }
      if (msg == 'h') {
        Serial.println("Received handshake msg, switching to handshake protocol");
        initialize();
      }
    }
    else {
      Serial.println("unexpected error in transmitData");
      //sc_pos_json = sc_pos_backup;
    }

    //xSemaphoreGive(xSemaphore);
    vTaskDelay(50);
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

  motorPins[0] = motorPin1;
  motorPins[1] = motorPin2;
  motorPins[2] = motorPin3;
  motorPins[3] = motorPin4;
  motorPins[4] = motorPin5;
  

  //NewPing Sensors
  pingTimer[0] = millis() + 75; // First ping start in ms.
  for (uint8_t i = 1; i < SONAR_NUM; i++) {
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
  }
    
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
  compass.m_min = (LSM303::vector<int16_t> ) {
    -32767, -32767, -32767
  };
  compass.m_max = (LSM303::vector<int16_t> ) {
    +32767, +32767, +32767
  };

  calibrate();

  //  ===============================  Create Hardware Tasks  ===============================
  xTaskCreate(getAccelReadings, "getAccelReadings", 3 * STACK_SIZE, NULL, 3, NULL);
  xTaskCreate(getSensorReadings, "getSensorReadings", STACK_SIZE, NULL, 3, NULL);

  //  ===============================  Setup Firmware Connection  ===============================
  if (!DEBUG_ARDUINO_ONLY) {
    Serial.println("Begin Arduino-Pi Connection");
    initialize();
    Serial.println("Connection Established. Sending data from Arduino to Pi.");
  } else {
    Serial.println("Arduino in testing mode - skipping handshake with Pi");
  }

  //  ===============================  Create Firmware Tasks  ===============================
  //	xTaskCreate(addStepCountDataDebug, "addStepCountData", STACK_SIZE, NULL, 1, NULL); // Debugging Purpose
  if (!DEBUG_ARDUINO_ONLY) xTaskCreate(transmitStepCountData, "transmitStepCountData", STACK_SIZE, NULL, 2, NULL); // Keep Transmit at Lower Priority than Sensor Readings

  vTaskStartScheduler();
}

void loop() {

}
