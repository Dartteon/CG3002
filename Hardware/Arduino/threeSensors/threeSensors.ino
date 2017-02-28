#include <Wire.h>
#include <LSM303.h>
LSM303 compass;

#define uSoundTrigPin_LeftShoulder 45
#define uSoundEchoPin_LeftShoulder 44
#define motor_LeftShoulder 47

#define uSoundTrigPin_Mid 49
#define uSoundEchoPin_Mid 48
#define motor_Mid 51

#define uSoundTrigPin_RightShoulder 33
#define uSoundEchoPin_RightShoulder 32
#define motor_RightShoulder 34

int distThreshold_Sides = 50;
int distThreshold_Mid = 100;

bool READ_LEFT_SHOULDER = true;
bool READ_MID = false;
bool READ_RIGHT_SHOULDER = false;
bool READ_COMPASS = false;
int phase = 0;

void setup() {
  Serial.begin (9600);

  //Compass
  Wire.begin();
  compass.init();
  compass.enableDefault();
  compass.m_min = (LSM303::vector<int16_t>) {
    -32767, -32767, -32767
  };
  compass.m_max = (LSM303::vector<int16_t>) {
    +32767, +32767, +32767
  };

  pinMode(uSoundTrigPin_LeftShoulder, OUTPUT);
  pinMode(uSoundTrigPin_Mid, OUTPUT);
  pinMode(uSoundTrigPin_RightShoulder, OUTPUT);

  pinMode(uSoundEchoPin_LeftShoulder, INPUT);
  pinMode(uSoundEchoPin_Mid, INPUT);
  pinMode(uSoundEchoPin_RightShoulder, INPUT);

  pinMode(motor_LeftShoulder, OUTPUT);
  pinMode(motor_Mid, OUTPUT);
  pinMode(motor_RightShoulder, OUTPUT);
}

void loop() {
  switch(phase) {
    case 0: readLeftShoulder();
    break;
    case 1: readRightShoulder();
    break;
    case 2: //readMid();
    break;
    case 3: readHeading();
    break;
  }
  phase = (phase + 1) % 4;
  delay(100);
}

void readLeftShoulder() {
  long durationLeftShoulder;
  long distanceLeftShoulder = 9999;
  digitalWrite(uSoundTrigPin_LeftShoulder, LOW);
  delayMicroseconds(2);
  digitalWrite(uSoundTrigPin_LeftShoulder, HIGH);
  delayMicroseconds(10); // Added this line
  digitalWrite(uSoundTrigPin_LeftShoulder, LOW);
  durationLeftShoulder = pulseIn(uSoundEchoPin_LeftShoulder, HIGH);
  distanceLeftShoulder = (durationLeftShoulder / 2) / 29.1;
//    Serial.print("L ");
//    Serial.print(distanceLeftShoulder);
//    Serial.println(" cm");
  digitalWrite(motor_LeftShoulder, (distanceLeftShoulder <= distThreshold_Sides) ? HIGH : LOW);
}
void readRightShoulder() {

  long durationRightShoulder;
  long distanceRightShoulder = 9999;
  digitalWrite(uSoundTrigPin_RightShoulder, LOW);
  delayMicroseconds(2); // Added this line
  digitalWrite(uSoundTrigPin_RightShoulder, HIGH);
  delayMicroseconds(10); // Added this line
  digitalWrite(uSoundTrigPin_RightShoulder, LOW);
  durationRightShoulder = pulseIn(uSoundEchoPin_RightShoulder, HIGH);
  distanceRightShoulder = (durationRightShoulder / 2) / 29.1;
    Serial.print("R ");
    Serial.print(distanceRightShoulder);
    Serial.println(" cm");
  digitalWrite(motor_RightShoulder, (distanceRightShoulder <= distThreshold_Sides) ? HIGH : LOW);
}
void readMid() {
  long durationMid;
  long distanceMid = 9999;
  digitalWrite(uSoundTrigPin_Mid, LOW);
  delayMicroseconds(2);
  digitalWrite(uSoundTrigPin_Mid, HIGH);
  delayMicroseconds(10);
  digitalWrite(uSoundTrigPin_Mid, LOW);
  durationMid = pulseIn(uSoundEchoPin_Mid, HIGH);
  distanceMid = (durationMid / 2) / 29.1;
//    Serial.print("M ");
//    Serial.print(distanceMid);
//    Serial.println(" cm");
  digitalWrite(motor_Mid, (distanceMid <= distThreshold_Mid) ? HIGH : LOW);
}

void readHeading() {

  compass.read();
  float heading = compass.heading((LSM303::vector<int>) {
    0, 0, 1
  });
//  Serial.println(heading);
}

