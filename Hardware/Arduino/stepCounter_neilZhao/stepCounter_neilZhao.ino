/*
    Here is a sketch that "graphs" the accelerometer and gyroscope data coming from
    the Pololu L3G and LSM303 libraries--by rapidly outputting graphically-arranged
    strings to the Arduino serial monitor. It's much easier than strings of numbers
    to look at and tell whether your sensors are working properly.
*/

#include <Wire.h>
#include <String.h>
#include <L3G.h>
#include <LSM303.h>

//Sensor libraries
LSM303 compass;
L3G gyro;

//Variables
int currGyroX, currGyroY, currGyroZ;
int xFilter[4] = { 0 };
int xSamples[50] = { 0 };
int xDynamicThreshold = 0;
int xGyroOffset, yGyroOffset, zGyroOffset;
int currSampleCount = 0;

int numStepsTaken = 0;
int MINIMUM_ACCELERATION_DELTA = 300; //Crossing below threshold not enough - it must be a decent acceleration change
unsigned long lastStepTime;

bool hasInit = false;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  if (!gyro.init())
  {
    Serial.println("Failed to autodetect gyro type!");
    while (1);
  }
  if (!compass.init())
  {
    Serial.println("Failed to initialize compass!");
    while (1);
  }
  gyro.enableDefault();
  compass.enableDefault();
  calibrate();
  hasInit = true;
}

void loop() {
  if (!hasInit) return;
  int prevSample = currGyroX; //Get previous reading
  readAltimu();
  int diff = abs(prevSample - currGyroX);
  
  unsigned long currTime = millis();
  unsigned long timeDiff = currTime - lastStepTime;

  if (diff < MINIMUM_ACCELERATION_DELTA) return;
  //  Serial.println("Diff = " + (String)diff);
  xSamples[currSampleCount] = currGyroX;
  if (timeDiff >= 1000) {
    if (currGyroX < xDynamicThreshold) {
      //      incrementSteps(); 
      lastStepTime = currTime;
      numStepsTaken++;
      Serial.println("Step taken! Total steps - " + (String)numStepsTaken + " ---- CurrGyroZ = ");
      Serial.println("TimeDiff = " + (String)timeDiff);
    }
  }

  incrementSampleCount();

  delay(20);
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
  //  currGyroX = (int) gyro.g.x - xGyroOffset;
  //  currGyroY = (int) gyro.g.y - yGyroOffset;
  //  currGyroZ = (int) gyro.g.z - zGyroOffset;
  currGyroX = (int) compass.a.x - xGyroOffset;
  currGyroY = (int) compass.a.y - yGyroOffset;
  currGyroZ = (int) compass.a.z - zGyroOffset;
//  compass.g.x
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
  int xMax = xSamples[0];
  int xMin = xSamples[0];
  for (int i = 0; i < 50; i++) {
    if (xSamples[i] > xMax) xMax = xSamples[i];
    if (xSamples[i] < xMin) xMin = xSamples[i];
  }
  xDynamicThreshold = (xMax + xMin) / 2;
}

