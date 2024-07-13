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
        // Read data including checksum
        uint8_t buffer[NUM_THRUSTERS * sizeof(uint16_t) + 1];
        for (size_t i = 0; i < sizeof(buffer); i++) {
            buffer[i] = Serial.read();
        }

        // Calculate expected checksum (xor of data bytes)
        uint8_t expected_checksum = 0;
        for (size_t i = 0; i < sizeof(buffer) - 1; i++) {
            expected_checksum ^= buffer[i];
        }

        uint8_t received_checksum = buffer[sizeof(buffer) - 1];

        if (received_checksum == expected_checksum) {
            // Checksum is valid, process the received data
            for (uint8_t i = 0; i < NUM_THRUSTERS; i++) {
                uint16_t val = buffer[2*i] | (buffer[2*i + 1] << 8); // Convert bytes to uint16_t (little endian)

                pwms[i] = val;
            }
        } else {
            // Checksum mismatch, stop all thrusters
            for (uint8_t i = 0; i < NUM_THRUSTERS; i++) {
                pwms[i] = THRUSTER_STOP_PWM;
            }
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