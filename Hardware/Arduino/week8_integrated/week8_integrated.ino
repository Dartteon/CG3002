#include <Wire.h>
#include <LSM303.h>
#include <L3G.h>
LSM303 compass;
L3G gyro;

//  ============================  Ultrasound Sensors  ============================
//Left
#define trigPin1 45
#define echoPin1 44
#define motorPin1 47
//Mid
#define trigPin2 49
#define echoPin2 48
#define motorPin2 51
//Right
#define trigPin3 26
#define echoPin3 27
#define motorPin3 28
long duration, distance, RightSensor, BackSensor, FrontSensor, LeftSensor;
int DIST_THRESHOLD_SIDES = 50;
int DIST_THRESHOLD_MID = 50;
int DURATION_TIMEOUT_SENSOR = 3000;
//  ==============================================================================

//  ===============================  Step Counter  ===============================
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

void setup()
{
  Serial.begin (115200);
    Serial.println("Initializing!");
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(trigPin3, OUTPUT);
  pinMode(echoPin3, INPUT);
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(motorPin3, OUTPUT);


  //  Compass
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

//No need gyro.. for now
//  if (!gyro.init())
//  {
//    Serial.println("Failed to autodetect gyro type!");
//    while (1);
//  } else {
//    Serial.println("Gyro Initialized!");
//  }
//  gyro.enableDefault();
  compass.enableDefault();
  compass.m_min = (LSM303::vector<int16_t>) {
    -32767, -32767, -32767
  };
  compass.m_max = (LSM303::vector<int16_t>) {
    +32767, +32767, +32767
  };
  calibrate();
}

void loop() {
  if (!proceed) return;
  switch (phase) {
    case 0: 
      readSensor(0);
      break;
    case 2: 
      readSensor(1);
      break;
    case 4: 
      readSensor(2);
      break;
    case 6: 
      readCompass();
      break;
    default:
      readStepCounter();
      break;
  }
  phase = (phase + 1) % 50;
}

void readSensor(int i) {
  switch (i) {
    case 0:
      SonarSensor(trigPin1, echoPin1);
      LeftSensor = distance;
      digitalWrite(motorPin1, (LeftSensor <= DIST_THRESHOLD_SIDES) ? HIGH : LOW);
//     Serial.print(LeftSensor); Serial.print(" - ");
      break;
    case 1:
      SonarSensor(trigPin2, echoPin2);
      FrontSensor = distance;
      digitalWrite(motorPin2, (FrontSensor <= DIST_THRESHOLD_MID) ? HIGH : LOW);
//      Serial.print(FrontSensor); Serial.print(" - ");
      break;
    case 2:
      SonarSensor(trigPin3, echoPin3);
      RightSensor = distance;
      digitalWrite(motorPin3, (RightSensor <= DIST_THRESHOLD_SIDES) ? HIGH : LOW);
//      Serial.println(RightSensor);
      break;
  }
}

void readCompass() {
  compass.read();
  float heading = compass.heading((LSM303::vector<int>) {
    0, 0, 1
  });
//  Serial.println(heading);
}

void SonarSensor(int trigPin, int echoPin)
{
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

// ================= Accelerometer


void readStepCounter() {
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
//      Serial.println("Step detected but not within interval threshold");
  }

}

void incrementSampleCount() {
  if (currSampleCount >= NUM_SAMPLE_COUNTS_TO_RECALCULATE_THRESHOLD - 1) {
    calculateNewXThreshold(); //Set new threshold
    currSampleCount = 0;
  }
  else currSampleCount++;
}
void readAltimu() {
  compass.read();
  xSampleOld = xSampleNew;  //Compulsory shift in
  int newAccX = (int) compass.a.x - xAccOffset;
  int diff = abs(newAccX - xSampleOld);
  if (diff >= PREDIFINED_PRECISION) { //delta is significant enough to shift in
//    xSampleNew = (xFilter[0] + xFilter[1] + xFilter[2] + newAccX)/4.0;  //Averaging over past 3 readings
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
//    Serial.println("Gyro " + (String)gyroX + " " + (String)currGyroY + " " + (String)currGyroZ);
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
void calculateNewXThreshold() {
  xDynamicThreshold = (xMax + xMin) / 2;
//  Serial.print("xDynamicThreshold = "); Serial.println(xDynamicThreshold);
  xMax = -9999;
  xMin = 9999;
}
