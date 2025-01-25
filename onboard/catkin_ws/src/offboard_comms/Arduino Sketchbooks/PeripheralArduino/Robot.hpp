#include <vector>

#define extractOpCode(x) (x & 0b11100000) >> 5
#define extractServoPin(x) (x & 0b00011110) >> 1
#define extractDirection(x) (x & 0b00000001)



class Robot {
    private:
        bool isShell;
        std::vector<Servo> servoList;
        

    public:
        Robot(bool isShell = false) : isShell(isShell) {}
        
        void init();
        void process() {
            if (Serial.available() > 0) {
                byte input = (byte) Serial.read();
            
                byte command = extractOpCode(input);
                byte pin = extractServoPin(input);
                byte direction = extractDirection(input);



                if (Serial.available() > 0) {
                    Serial.readString();
                }
            }

            for (Servo servo : servoList) {
                servo.callServo();
            }
        }
}