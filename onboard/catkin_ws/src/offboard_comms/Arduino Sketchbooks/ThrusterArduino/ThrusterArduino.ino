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

void write_pwms()
{
    bool timeout = last_cmd_ms_ts + THRUSTER_TIMEOUT_MS < millis();

    for (uint8_t i = 0; i < NUM_THRUSTERS; ++i)
    {
        uint16_t pwm = timeout ? THRUSTER_STOP_PWM : pwms[i];

        // If PWM is out of bounds, stop thruster
        if (pwm < THRUSTER_PWM_MIN || pwm > THRUSTER_PWM_MAX)
        {
            pwm = THRUSTER_STOP_PWM;
        }

        thrusters[i].write(pwm + THRUSTER_PWM_OFFSET);
    }
}

void setup()
{
    // Set all PWMs to stop for proper initialization of thrusters
    for (uint8_t i = 0; i < NUM_THRUSTERS; ++i)
    {
        pwms[i] = THRUSTER_STOP_PWM;
    }

    pwm_multiplexer.begin();
    for (uint8_t i = 0; i < NUM_THRUSTERS; ++i)
    {
        thrusters[i].initialize(&pwm_multiplexer);
        thrusters[i].attach(i);
    }

    // Write the 1500 PWM to all thrusters to stop them
    for (uint8_t i = 0; i < NUM_THRUSTERS; ++i)
        thrusters[i].write(THRUSTER_STOP_PWM + THRUSTER_PWM_OFFSET);

    Serial.begin(BAUD_RATE);
}

void loop()
{
    // Check if last version of data has timed out, if so, stop all thrusters

    // Read from serial as a string of 8 integers separated by commas
    // Ex.
    // 1496,1497,1498,1499,1500,1501,1502,1503

    if (Serial.available() > 0)
    {
        String input = Serial.readStringUntil('\n');
        int i = 0;
        int j = 0;
        while (input.length() > 0)
        {
            int comma = input.indexOf(',');
            if (comma == -1)
            {
                uint16_t val = input.toInt();
                if (val < THRUSTER_PWM_MIN || val > THRUSTER_PWM_MAX)
                {
                    val = THRUSTER_STOP_PWM;
                }
                break;
            }
            else
            {
                uint16_t val = input.substring(0, comma).toInt();
                if (val < THRUSTER_PWM_MIN || val > THRUSTER_PWM_MAX)
                {
                    val = THRUSTER_STOP_PWM;
                }
                pwms[j] = val;
                input = input.substring(comma + 1);
            }
            j++;
        }
        last_cmd_ms_ts = millis();
        pwms_changed = true;
    }

    // Write the PWMs to the thrusters if they have changed
    if (pwms_changed)
    {
        write_pwms();
        pwms_changed = false;
    }

}
