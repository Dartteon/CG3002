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
int currGyroX, currGyroY, currGyroZ;
int xFilter[4] = { 0 };
int xSamples[50] = { 0 };
int xDynamicThreshold = 0;
int xGyroOffset, yGyroOffset, zGyroOffset;
int currSampleCount = 0;
int xMin = 0, xMax =-0;

int numStepsTaken = 0;
int MINIMUM_ACCELERATION_Z = 400;
int MINIMUM_STEP_INTERVAL = 30;
int MINIMUM_ACCELERATION_DELTA = 800; //Crossing below threshold not enough - it must be a decent acceleration change
unsigned long lastStepTime;
//  ==============================================================================
bool proceed = true;

int phase = 0;

void setup()
{
  Serial.begin (115200);
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
  compass.init();
  delayMicroseconds(10);
  gyro.init();
  delayMicroseconds(10);
  
  if (!compass.init())
  {
    Serial.println("Failed to initialize compass!");
    while (1);
  } else {
    Serial.println("Compass Initialized!");
  }

  if (!gyro.init())
  {
    Serial.println("Failed to autodetect gyro type!");
    while (1);
  } else {
    Serial.println("Gyro Initialized!");
  }
  gyro.enableDefault();
  compass.enableDefault();
  compass.m_min = (LSM303::vector<int16_t>) {
    -32767, -32767, -32767
  };
  compass.m_max = (LSM303::vector<int16_t>) {
    +32767, +32767, +32767
  };
  calibrate();
}

int delayTime = 200;
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
//      Serial.print(LeftSensor);
//      Serial.print(" - ");
      break;
    case 1:
      SonarSensor(trigPin2, echoPin2);
      FrontSensor = distance;
      digitalWrite(motorPin2, (FrontSensor <= DIST_THRESHOLD_MID) ? HIGH : LOW);
//      Serial.print(FrontSensor);
//      Serial.print(" - ");
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
  Serial.println(heading);
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
  int prevSample = currGyroX; //Get previous reading
  readAltimu();
  int diff = abs(prevSample - currGyroX);
  
  unsigned long currTime = millis();
  unsigned long timeDiff = currTime - lastStepTime;

  if (diff < MINIMUM_ACCELERATION_DELTA) return;
//    Serial.println("Diff = " + (String)diff);
  if (timeDiff < MINIMUM_STEP_INTERVAL) return;
  if (currGyroZ < MINIMUM_ACCELERATION_Z) return;
  xSamples[currSampleCount] = currGyroX;
  if (currGyroX > xMax) xMax = currGyroX;
  if (currGyroX < xMin) xMin = currGyroX;
  if (timeDiff >= 1000) {
    if (currGyroX < xDynamicThreshold) {
      //      incrementSteps(); 
      lastStepTime = currTime;
      numStepsTaken++;
      Serial.println("Step taken! Total steps - " + (String)numStepsTaken + " ---- CurrGyroZ = ");
//      Serial.println(currGyroZ);
//      Serial.println("TimeDiff = " + (String)timeDiff);
    }
  } else {
      Serial.println("Step detected but not within interval threshold");
  }

  incrementSampleCount();
}

void incrementSampleCount() {
  if (currSampleCount >= 49) {
    calculateNewXThreshold(); //Set new threshold
    currSampleCount = 0;
  }
  else currSampleCount++;
  //  Serial.println("CurrCount " + (String)currSampleCount);
}
void readAltimu() {
  gyro.read();
  compass.read();
  currGyroX = (int) compass.a.x - xGyroOffset;
  currGyroY = (int) compass.a.y - yGyroOffset;
  currGyroZ = (int) compass.a.z - zGyroOffset;

  xFilter[3] = xFilter[2];
  xFilter[2] = xFilter[1];
  xFilter[1] = xFilter[0];
  xFilter[0] = currGyroX;
  currGyroX = (xFilter[0] + xFilter[1] + xFilter[2] + xFilter[3])/4.0;

//    Serial.println("Gyro " + (String)currGyroX + " " + (String)currGyroY + " " + (String)currGyroZ);

}
void calibrate() {
  int NUM_SAMPLES = 128;
  long xGyroSum = 0;
  long yGyroSum = 0;
  long zGyroSum = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) {
    readAltimu();
    xGyroSum += currGyroX;
    yGyroSum += currGyroY;
    zGyroSum += currGyroZ;
  }
  xGyroOffset = xGyroSum / NUM_SAMPLES;
  yGyroOffset = yGyroSum / NUM_SAMPLES;
  zGyroOffset = zGyroSum / NUM_SAMPLES;
  Serial.println("Calibrated " + (String)xGyroOffset + " "
                 + (String)yGyroOffset + " " +  (String)zGyroOffset);
                 currGyroX = 0;
}
void calculateNewXThreshold() {
  /*int xMax = xSamples[0];
  int xMin = xSamples[0];
  for (int i = 0; i < 50; i++) {
    if (xSamples[i] > xMax) xMax = xSamples[i];
    if (xSamples[i] < xMin) xMin = xSamples[i];
  }*/
  xDynamicThreshold = (xMax + xMin) / 2;
  xMax=-9999;
  xMin=9999;
}
