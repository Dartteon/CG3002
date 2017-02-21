#define uSoundTrigPin_LeftShoulder 50
#define uSoundEchoPin_LeftShoulder 51
#define motor_LeftShoulder 53

#define uSoundTrigPin_Mid 23
#define uSoundEchoPin_Mid 22
#define motor_Mid 25

#define uSoundTrigPin_RightShoulder 33
#define uSoundEchoPin_RightShoulder 32
#define motor_RightShoulder 37

int distThreshold_Sides = 50;
int distThreshold_Mid = 50;

bool READ_LEFT_SHOULDER = true;
bool READ_MID = false;
bool READ_RIGHT_SHOULDER = false;
int phase = 0;

void setup() {
  Serial.begin (9600);
  
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

void setPhase() {
  READ_LEFT_SHOULDER = (phase == 0);
  READ_MID = (phase == 1);
  READ_RIGHT_SHOULDER = (phase == 2);
  phase = (phase + 1) % 3;
}

void loop() {
  setPhase();
  
  long durationLeftShoulder, durationRightShoulder, durationMid;
  long distanceLeftShoulder, distanceRightShoulder, distanceMid;

  if (READ_LEFT_SHOULDER) digitalWrite(uSoundTrigPin_LeftShoulder, LOW);
  if (READ_MID) digitalWrite(uSoundTrigPin_Mid, LOW);
  if (READ_RIGHT_SHOULDER) digitalWrite(uSoundTrigPin_RightShoulder, LOW);

  delayMicroseconds(2); // Added this line
  
  if (READ_LEFT_SHOULDER) digitalWrite(uSoundTrigPin_LeftShoulder, HIGH);
  if (READ_MID) digitalWrite(uSoundTrigPin_Mid, HIGH);
  if (READ_RIGHT_SHOULDER) digitalWrite(uSoundTrigPin_RightShoulder, HIGH);

  delayMicroseconds(10); // Added this line
  
  if (READ_LEFT_SHOULDER) digitalWrite(uSoundTrigPin_LeftShoulder, LOW);
  if (READ_MID) digitalWrite(uSoundTrigPin_Mid, LOW);
  if (READ_RIGHT_SHOULDER) digitalWrite(uSoundTrigPin_RightShoulder, LOW);
  
  if (READ_LEFT_SHOULDER) durationLeftShoulder = pulseIn(uSoundEchoPin_LeftShoulder, HIGH);
  if (READ_MID) durationMid = pulseIn(uSoundEchoPin_Mid, HIGH);
  if (READ_RIGHT_SHOULDER) durationRightShoulder = pulseIn(uSoundEchoPin_RightShoulder, HIGH);
  
  if (READ_LEFT_SHOULDER) distanceLeftShoulder = (durationLeftShoulder/2) / 29.1;
  if (READ_MID) distanceMid = (durationMid/2) / 29.1;
  if (READ_RIGHT_SHOULDER) distanceRightShoulder = (durationRightShoulder/2) / 29.1;
  
//  if (READ_LEFT_SHOULDER) {
//    Serial.print("Left ");
//    Serial.print(distanceLeftShoulder);
//    Serial.println(" cm");
//  }
  if (READ_MID) {
    Serial.print("Mid ");
    Serial.print(distanceMid);
    Serial.println(" cm");
  }
//  if (READ_RIGHT_SHOULDER) {
//    Serial.print("Right ");
//    Serial.print(distanceRightShoulder);
//    Serial.println(" cm");
//  }
  
  if (READ_LEFT_SHOULDER) digitalWrite(motor_LeftShoulder, (distanceLeftShoulder <= distThreshold_Sides) ? HIGH: LOW);
  if (READ_MID) digitalWrite(motor_Mid, (distanceMid <= distThreshold_Sides) ? HIGH: LOW);
  if (READ_RIGHT_SHOULDER) digitalWrite(motor_RightShoulder, (distanceRightShoulder <= distThreshold_Sides) ? HIGH: LOW);
}
