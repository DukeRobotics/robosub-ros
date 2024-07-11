#include "Adafruit_PWMServoDriver.h"
#include "MultiplexedBasicESC.h"
#include <Arduino.h>
#include "offset.h"

Adafruit_PWMServoDriver pwm_multiplexer(0x40);

#define BAUD_RATE 57600
#define NUM_THRUSTERS 8
#define THRUSTER_TIMEOUT_MS 500
#define THRUSTER_STOP_PWM 1500
#define THRUSTER_PWM_MIN 1100
#define THRUSTER_PWM_MAX 1900

extern int THRUSTER_PWM_OFFSET; // Hardware specific offset for PWMs -- refers to the robot-specific offsets

uint64_t last_cmd_ms_ts;

uint16_t pwms[NUM_THRUSTERS];

MultiplexedBasicESC thrusters[NUM_THRUSTERS];

bool pwms_changed = true;

void write_pwms() {

    if (last_cmd_ms_ts + THRUSTER_TIMEOUT_MS < millis()) {
        // Stop all thrusters if no command has been received in the last THRUSTER_TIMEOUT_MS milliseconds
        for (uint8_t i = 0; i < NUM_THRUSTERS; i++) {
            thrusters[i].write(THRUSTER_STOP_PWM + THRUSTER_PWM_OFFSET);
        }
    } else {
        for (uint8_t i = 0; i < NUM_THRUSTERS; i++) {
            uint16_t pwm_value = pwms[i];

            // Clamp the PWM value to be within bounds
            // We optionally can stop the thruster if the PWM value is out of bounds, but this way improves performance
            pwm_value = max(THRUSTER_PWM_MIN, min(pwm_value, THRUSTER_PWM_MAX));

            thrusters[i].write(pwm_value + THRUSTER_PWM_OFFSET);
        }
    }
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

    // Write the stop PWM to all thrusters
    for (uint8_t i = 0; i < NUM_THRUSTERS; i++) {
        thrusters[i].write(THRUSTER_STOP_PWM + THRUSTER_PWM_OFFSET);
    }

    Serial.begin(BAUD_RATE);
}

void loop() {
    static char inputBuffer[64];
    static uint8_t inputPos = 0;

    if (Serial.available() > 0) {
        while (Serial.available() > 0) {
            char c = Serial.read();
            if (c == '\n' || inputPos >= sizeof(inputBuffer) - 1) {
                inputBuffer[inputPos] = '\0';
                inputPos = 0;

                // Parse the inputBuffer
                char *token = strtok(inputBuffer, ",");
                uint8_t index = 0;
                while (token != NULL && index < NUM_THRUSTERS) {
                    uint16_t val = atoi(token);
                    if (val < THRUSTER_PWM_MIN || val > THRUSTER_PWM_MAX) {
                        val = THRUSTER_STOP_PWM;
                    }
                    pwms[index] = val;
                    token = strtok(NULL, ",");
                    index++;
                }
                last_cmd_ms_ts = millis();
                pwms_changed = true;
                break;
            } else {
                inputBuffer[inputPos++] = c;
            }
        }
    }

    // Write the PWMs to the thrusters if they have changed
    if (pwms_changed) {
        write_pwms();
        pwms_changed = false;
    }
}
