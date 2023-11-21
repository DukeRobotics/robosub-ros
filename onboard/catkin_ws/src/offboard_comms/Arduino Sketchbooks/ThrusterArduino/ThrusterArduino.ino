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

uint64_t last_cmd_ms_ts;

int16_t pwms[NUM_THRUSTERS];

MultiplexedBasicESC thrusters[NUM_THRUSTERS];

// Sets node handle to have 1 subscriber, 1 publisher, and 128 bytes for input and output buffer
ros::NodeHandle_<ArduinoHardware,1,1,128,128> nh;

// Reusing ESC library code

void thruster_pwm_callback(const custom_msgs::PWMAllocs &pwm_msg){
    // Copy the contents of the pwm message to the local array
    memcpy(pwms, pwm_msg.allocs, sizeof(pwms));
    last_cmd_ms_ts = millis();
}

ros::Subscriber<custom_msgs::PWMAllocs> ts_sub("/offboard/pwm", &thruster_pwm_callback);

void setup(){
    for (uint8_t i = 0; i < NUM_THRUSTERS; ++i)
        pwms[i] = THRUSTER_STOP_PWM;

    Serial.begin(BAUD_RATE);
    nh.getHardware()->setBaud(BAUD_RATE);
    nh.initNode();
    nh.subscribe(ts_sub);

    pwm_multiplexer.begin();
    for (uint8_t i = 0; i < NUM_THRUSTERS; ++i){
        thrusters[i].initialize(&pwm_multiplexer);
        thrusters[i].attach(i);
    }
}

void loop(){
    // Check if last version of data has timed out, if so, stop all thrusters
    bool timeout = last_cmd_ms_ts + THRUSTER_TIMEOUT_MS < millis();
    for (uint8_t i = 0; i < NUM_THRUSTERS; ++i) {
        int16_t pwm = timeout ? THRUSTER_STOP_PWM : pwms[i];
        thrusters[i].write(pwm + THRUSTER_PWM_OFFSET);
    }

	nh.spinOnce();
}
