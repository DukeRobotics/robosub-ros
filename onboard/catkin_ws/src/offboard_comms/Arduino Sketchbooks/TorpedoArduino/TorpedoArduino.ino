#include <Servo.h>

Servo myservo;

unsigned long myTime;
boolean servoMoved = false;
unsigned long servoTime;
unsigned long rateServo = 1000;  // how much to wait before returning servo to default position

void setup() {
  Serial.begin(9600);
  myservo.attach(9);  // servo pin 9
  myservo.write(90);
}

void loop() {
  myTime = millis();
  
  if (Serial.available()>0 && !servoMoved) {
    char state = Serial.read();
    if (Serial.available() > 0) {
      Serial.readString();
    }

    if (state == 'L') {
        myservo.write(160);
        servoMoved = true;
        servoTime = myTime;
      }
      if (state == 'R') {
        myservo.write(20);
        servoMoved = true;
        servoTime = myTime;
      }
  }
  
  if(servoMoved && myTime - servoTime > rateServo) {
      myservo.write(90);
      servoMoved = false;
    }
}