#include "Adafruit_PWMServoDriver.h"
#include "MultiplexedServo.h"
#include "MultiplexedBasicESC.h"
#include <ros.h>
#include <custom_msgs/PWMAllocs.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Bool.h>
#include <Arduino.h>

Adafruit_PWMServoDriver pwm_multiplexer(0x40);

#define BAUD_RATE 57600
#define NUM_THRUSTERS 8
#define THRUSTER_TIMEOUT_MS 500
#define THRUSTER_STOP_PWM 1500
#define THRUSTER_PWM_OFFSET 31 // Apply offset of +31 for oogway, remove for cthulhu
#define THRUSTER_PWM_MIN 1100
#define THRUSTER_PWM_MAX 1900

uint64_t last_cmd_ms_ts;

int16_t pwms[NUM_THRUSTERS];

MultiplexedBasicESC thrusters[NUM_THRUSTERS];

// Sets node handle to have 0 publishers, 1 subscriber, and 128 bytes for input and output buffer
ros::NodeHandle_<ArduinoHardware, 0, 1, 128, 128> nh;

// Reusing ESC library code

void thruster_pwm_callback(const custom_msgs::PWMAllocs &pwm_msg)
{
    // Copy the contents of the pwm message to the local array
    if (pwm_msg.allocs_length != NUM_THRUSTERS)
    {
        String msg = "Received PWM message with incorrect number of allocations. Recieved: " +
                     String(pwm_msg.allocs_length) + ", expected: " + String(NUM_THRUSTERS) + ".";
        nh.logerror(msg.c_str());
        return;
    }

    memcpy(pwms, pwm_msg.allocs, sizeof(pwms));
    last_cmd_ms_ts = millis();
}

ros::Subscriber<custom_msgs::PWMAllocs> ts_sub("/offboard/pwm", &thruster_pwm_callback);

void setup()
{
    for (uint8_t i = 0; i < NUM_THRUSTERS; ++i)
        pwms[i] = THRUSTER_STOP_PWM;

    Serial.begin(BAUD_RATE);
    nh.getHardware()->setBaud(BAUD_RATE);
    nh.initNode();
    nh.subscribe(ts_sub);

    pwm_multiplexer.begin();
    for (uint8_t i = 0; i < NUM_THRUSTERS; ++i)
    {
        thrusters[i].initialize(&pwm_multiplexer);
        thrusters[i].attach(i);
    }
}

void loop()
{
    // Check if last version of data has timed out, if so, stop all thrusters
    bool timeout = last_cmd_ms_ts + THRUSTER_TIMEOUT_MS < millis();

    for (uint8_t i = 0; i < NUM_THRUSTERS; ++i)
    {
        int16_t pwm = timeout ? THRUSTER_STOP_PWM : pwms[i];

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
