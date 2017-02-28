#include <Wire.h>
#include <LSM303.h>
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
#define trigPin3 33
#define echoPin3 32
#define motorPin3 34
long duration, distance, RightSensor, BackSensor, FrontSensor, LeftSensor;
int DIST_THRESHOLD_SIDES = 50;
int DIST_THRESHOLD_MID = 50;
//  ==============================================================================


int phase = 0;

void setup()
{
  Serial.begin (9600);
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
  compass.init();
  compass.enableDefault();
  compass.m_min = (LSM303::vector<int16_t>) {
    -32767, -32767, -32767
  };
  compass.m_max = (LSM303::vector<int16_t>) {
    +32767, +32767, +32767
  };
}

int delayTime = 200;
void loop() {
  switch (phase) {
    case 0: 
      readSensor(0);
      break;
    case 1: 
      readSensor(1);
      break;
    case 2: 
      readSensor(2);
      break;
    case 3: 
      readCompass();
      break;
    default:
      readStepCounter();
      break;
  }
  phase = (phase + 1) % 8;
}

void readSensor(int i) {
  switch (i) {
    case 0:
      SonarSensor(trigPin3, echoPin3);
      LeftSensor = distance;
      digitalWrite(motorPin3, (LeftSensor <= DIST_THRESHOLD_SIDES) ? HIGH : LOW);
      Serial.print(LeftSensor);
      Serial.print(" - ");
      break;
    case 1:
      SonarSensor(trigPin1, echoPin1);
      FrontSensor = distance;
      digitalWrite(motorPin1, (FrontSensor <= DIST_THRESHOLD_MID) ? HIGH : LOW);
      Serial.print(FrontSensor);
      Serial.print(" - ");
      break;
    case 2:
      SonarSensor(trigPin2, echoPin2);
      RightSensor = distance;
      digitalWrite(motorPin2, (RightSensor <= DIST_THRESHOLD_SIDES) ? HIGH : LOW);
      Serial.println(RightSensor);
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

void readStepCounter() {
  //STUB
}

void SonarSensor(int trigPin, int echoPin)
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration / 2) / 29.1;

}
