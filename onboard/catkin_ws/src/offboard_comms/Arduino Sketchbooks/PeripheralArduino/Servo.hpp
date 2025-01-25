#include <Arduino.h>
#include <Servo.h>

class Servo {
    private:
        int pinNum;
        Servo myServo;
        bool servoMoved = false;
        unsigned long servoTime;
        float ONBOARD_VOLTAGE = 4.655;
        
    public:
        Servo(int pinNum) : pinNum(pinNum) {
            myServo.attach(pinNum);
            myServo.writeMicroseconds(1500);
            
        }

        void callServo() {
            unsigned long currentTime = millis();

            if (Serial.available() > 0 && !servoMoved) {
                char state = Serial.read();
                if (Serial.available() > 0) {
                    Serial.readString();
                }

                if (state == 'L') {
                    myservo.writeMicroseconds(1250);  // 1200 = 90 degrees left  // 1250 micros =  20 write
                    servoMoved = true;
                    servoTime = currentTime;
                }
                if (state == 'R') {
                    myservo.writeMicroseconds(1750);  // 1800 = 90 degrees right // 1750 micros = 160 write
                    servoMoved = true;
                    servoTime = currentTime;
                }
            }

            // return to default position after 1 second
            // if(servoMoved && myTime - servoTime > rateServo) {
            //     myservo.writeMicroseconds(1500);  // 1500 micros = 90 write
            //     servoMoved = false;
            // }
        }
}