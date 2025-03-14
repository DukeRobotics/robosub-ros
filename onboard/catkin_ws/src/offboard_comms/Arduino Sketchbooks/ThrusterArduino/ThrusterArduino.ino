#include "Adafruit_PWMServoDriver.h"
#include "MultiplexedBasicESC.h"
#include <ros.h>
#include <custom_msgs/PWMAllocs.h>
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

// Sets node handle to have 0 publishers, 1 subscriber, and 128 bytes for input and output buffer
ros::NodeHandle_<ArduinoHardware, 1, 1, 128, 128> nh;

// Reusing ESC library code

void thruster_pwm_callback(const custom_msgs::PWMAllocs &pwm_msg)
{
    // Check if allocations were received for the correct number of thrusters
    if (pwm_msg.allocs_length != NUM_THRUSTERS)
    {
        String msg = "Received PWM message with incorrect number of allocations. Recieved: " +
                     String(pwm_msg.allocs_length) + ", expected: " + String(NUM_THRUSTERS) + ".";
        nh.logerror(msg.c_str());
        return;
    }

    // Copy the contents of the pwm message to the local array
    memcpy(pwms, pwm_msg.allocs, sizeof(pwms));
    last_cmd_ms_ts = millis();
}

ros::Subscriber<custom_msgs::PWMAllocs> ts_sub("/offboard/pwm", &thruster_pwm_callback);

void setup()
{
    // Set all PWMs to stop for proper initialization of thrusters
    for (uint8_t i = 0; i < NUM_THRUSTERS; ++i)
        pwms[i] = THRUSTER_STOP_PWM;

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
    nh.getHardware()->setBaud(BAUD_RATE);
    nh.initNode();
    nh.subscribe(ts_sub);
}

void loop()
{
    // Check if last version of data has timed out, if so, stop all thrusters
    bool timeout = last_cmd_ms_ts + THRUSTER_TIMEOUT_MS < millis();

    for (uint8_t i = 0; i < NUM_THRUSTERS; ++i)
    {
        uint16_t pwm = timeout ? THRUSTER_STOP_PWM : pwms[i];

        // If PWM is out of bounds, log error and stop thruster
        if (pwm < THRUSTER_PWM_MIN || pwm > THRUSTER_PWM_MAX)
        {
            String msg = "Stopping thruster " + String(i) + ". PWM out of bounds: " + String(pwm);
            nh.logerror(msg.c_str());

            pwm = THRUSTER_STOP_PWM;
        }

        thrusters[i].write(pwm + THRUSTER_PWM_OFFSET);
    }

    nh.spinOnce();
}
