/*
 *  Here is a sketch that "graphs" the accelerometer and gyroscope data coming from
 *  the Pololu L3G and LSM303 libraries--by rapidly outputting graphically-arranged
 *  strings to the Arduino serial monitor. It's much easier than strings of numbers
 *  to look at and tell whether your sensors are working properly.
 */

#include <Wire.h>
#include <String.h>
#include <L3G.h>
#include <LSM303.h>

LSM303 compass;
L3G gyro;

int lastGyroX, lastGyroY, lastGyroZ, lastCompassX, lastCompassY, lastCompassZ;
int currGyroX, currGyroY, currGyroZ;
int currCompassX, currCompassY, currCompassZ;
int xGyroAvg, yGyroAvg, zGyroAvg;

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
}

void loop() {
//  readAltimu();
//  int gyroX = currGyroX - xGyroAvg;
//  int gyroY = currGyroY - yGyroAvg;
//  int gyroZ = currGyroZ - zGyroAvg;
//
//  Serial.println("Gyro " + (String)gyroX + " " + (String)gyroY + " " + (String)gyroZ);
//  delay(20);
}

void readAltimu() {
  gyro.read();
  compass.read();
  currGyroX = (int) gyro.g.x;
  currGyroY = (int) gyro.g.y;
  currGyroZ = (int) gyro.g.z;
  currCompassX = (int) compass.a.x;
  currCompassY = (int) compass.a.y;
  currCompassZ = (int) compass.a.z;
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
    
    Serial.println("CurrSum " + (String)xGyroSum + " " 
      + (String)yGyroSum + " " +  (String)zGyroSum);
  }

  xGyroAvg = xGyroSum/NUM_SAMPLES;
  yGyroAvg = yGyroSum/NUM_SAMPLES;
  zGyroAvg = zGyroSum/NUM_SAMPLES;
  Serial.println("AverageGyro " + (String)xGyroAvg + " " 
    + (String)yGyroAvg + " " +  (String)zGyroAvg);

}

