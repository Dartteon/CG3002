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
int flag,steps,lowPassCount, lowPassThreshhold,reversePassCount, reversePassThreshhold;
float totave, totvect, totvect1;
float threshhold;

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
  flag = 0;
  threshhold = 85.0;
  steps = 0;
  lowPassCount = 0;
  lowPassThreshhold = 10;
  reversePassCount = 0;
  reversePassThreshhold = 10;
}

void loop() {
  readAltimu();
  int gyroX = currGyroX - xGyroAvg;
  int gyroY = currGyroY - yGyroAvg;
  int gyroZ = currGyroZ - zGyroAvg;

  //Serial.println("Gyro " + (String)gyroX + " " + (String)gyroY + " " + (String)gyroZ);

  
 if(gyroX < 0 ){
 totvect1 = -sqrt(gyroX * gyroX +  gyroZ * gyroZ);
 }else{
  totvect1 = sqrt(gyroX * gyroX +  gyroZ * gyroZ);
 }
 totave = (totvect + totvect1) / 2.0 ;
 totvect = totvect1;
 
 //Serial.println(totave);

//cal steps 
if (totave > threshhold && flag==0)
{
  //steps=steps+1;
  flag=1;
  lowPassCount = lowPassCount+ 1;
 
}
 else if (totave > threshhold && flag==1)
{
//do nothing 
  lowPassCount = lowPassCount+ 1;
  Serial.println(lowPassCount);
}else if ( flag==1 && lowPassCount > lowPassThreshhold && reversePassCount > reversePassThreshhold)
  {
    flag=0;
    steps = steps+1;
    lowPassCount = 0;
    reversePassCount = 0;
  }else if(totave < -threshhold && flag == 1 && lowPassCount <= lowPassThreshhold) {
    flag = 0;
    lowPassCount = 0;
  }else if(totave <-threshhold  && flag==1 && lowPassCount > lowPassThreshhold && reversePassCount <= reversePassThreshhold){
    reversePassCount = reversePassCount + 1;
  }else if(totave > threshhold && flag==1 && lowPassCount > lowPassThreshhold && reversePassCount >0 ){
    flag=0;
    
    lowPassCount = 0;
    reversePassCount = 0;
  } else if(totave < -threshhold && flag==0 && lowPassCount < lowPassThreshhold ){
    flag=0;
    
    lowPassCount = 0;
    reversePassCount = 0;
  }
  Serial.println('\n');
  Serial.print("steps=");
  Serial.println(steps);
  

 // delay(50);
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
  totvect = 0;
  totave = 0;
  Serial.println("AverageGyro " + (String)xGyroAvg + " " 
    + (String)yGyroAvg + " " +  (String)zGyroAvg);

}
