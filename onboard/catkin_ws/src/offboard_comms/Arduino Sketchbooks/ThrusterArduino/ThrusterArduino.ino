#include "Adafruit_PWMServoDriver.h"
#include "MultiplexedBasicESC.h"
#include <Arduino.h>
#include "offset.h"

Adafruit_PWMServoDriver pwm_multiplexer(0x40);

#define BAUD_RATE 57600
#define NUM_THRUSTERS 8
#define THRUSTER_TIMEOUT_MS 1000
#define THRUSTER_STOP_PWM 1500
#define THRUSTER_PWM_MIN 1100
#define THRUSTER_PWM_MAX 1900

extern int THRUSTER_PWM_OFFSET; // Hardware specific offset for PWMs -- refers to the robot-specific offsets

uint64_t last_cmd_ms_ts;

uint16_t pwms[NUM_THRUSTERS];

MultiplexedBasicESC thrusters[NUM_THRUSTERS];

void write_pwms() {

    for (uint8_t i = 0; i < NUM_THRUSTERS; i++) {
        // Clamp the PWM values to be within the min and max PWM values and write them to the thrusters
        thrusters[i].write(constrain(pwms[i], THRUSTER_PWM_MIN, THRUSTER_PWM_MAX) + THRUSTER_PWM_OFFSET);
    }
    last_cmd_ms_ts = millis();
}

void setup() {
    // Initialize the PWMs to stop
    for (uint8_t i = 0; i < NUM_THRUSTERS; i++) {
        pwms[i] = THRUSTER_STOP_PWM;
    }

    pwm_multiplexer.begin();
    for (uint8_t i = 0; i < NUM_THRUSTERS; i++) {
        thrusters[i].initialize(&pwm_multiplexer);
        thrusters[i].attach(i);
    }

    // Write the stop PWM to all thrusters to initialize them (proper beep sequence)
    for (uint8_t i = 0; i < NUM_THRUSTERS; i++) {
        thrusters[i].write(THRUSTER_STOP_PWM + THRUSTER_PWM_OFFSET);
    }

    Serial.begin(BAUD_RATE);
}

void loop() {
    // Only send new thruster values if we recieve new data -- the thrusters store the last command
    if (Serial.available() >= NUM_THRUSTERS * sizeof(uint16_t) + 1) {
        uint16_t allocs[NUM_THRUSTERS];
        byte incomingData[NUM_THRUSTERS * 2];
        byte startFlag = 0xFF;
        
        while (true) {
            if (Serial.available() > 0) {
            byte incomingByte = Serial.read();
            
            // Check if the received byte is the start flag (0xFF)
            if (incomingByte == startFlag) {
                break;  // Exit the loop once the start flag is detected
            }
            }
        }

        // Read the incoming data from the serial buffer (example: NUM_THRUSTERS * 2 bytes)
        size_t bytesRead = 0;
        while (bytesRead < sizeof(incomingData)) {
            incomingData[bytesRead] = Serial.read();
            bytesRead++;
        }

        // Now unpack the data from the byte array into the pwms array (big-endian)
        for (size_t i = 0; i < NUM_THRUSTERS; i++) {
            pwms[i] = incomingData[2 * i + 1] | (incomingData[2 * i] << 8);
        }
        
        write_pwms();
    }

    // If we haven't received a new command in a while, stop all thrusters
    else if (millis() - last_cmd_ms_ts > THRUSTER_TIMEOUT_MS) {
        for (uint8_t i = 0; i < NUM_THRUSTERS; i++) {
            pwms[i] = THRUSTER_STOP_PWM;
        }
        write_pwms();
    }
}