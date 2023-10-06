#include "Adafruit_PWMServoDriver.h"
#include "MultiplexedServo.h"
#include "MultiplexedBasicESC.h"
#include <ros.h>
#include "/root/dev/robosub-ros/onboard/catkin_ws/src/offboard_comms/Arduino Sketchbooks/ThrusterArduino/ros_lib/custom_msgs/ThrusterSpeeds.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Bool.h>
#include <Arduino.h>

Adafruit_PWMServoDriver pwm_multiplexer(0x40);

#define BAUD_RATE 57600
#define NUM_THRUSTERS 8
#define THRUSTER_TIMEOUT_MS 500

uint64_t last_cmd_ms_ts;

int8_t thruster_speeds[NUM_THRUSTERS];

MultiplexedBasicESC thrusters[NUM_THRUSTERS];

// Sets node handle to have 1 subscriber, 1 publisher, and 128 bytes for input and output buffer
ros::NodeHandle_<ArduinoHardware,1,1,128,128> nh;

// Reusing ESC library code
void thruster_speeds_callback(const custom_msgs::ThrusterSpeeds &ts_msg){
    // Copy the contents of the speed message to the local array
    memcpy(thruster_speeds, ts_msg.speeds, sizeof(thruster_speeds));
    last_cmd_ms_ts = millis();
}

ros::Subscriber<custom_msgs::ThrusterSpeeds> ts_sub("/offboard/thruster_speeds", &thruster_speeds_callback);

void setup(){
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
    // Check if last version of data has timed out, if so, reset the speeds
    if (last_cmd_ms_ts + THRUSTER_TIMEOUT_MS < millis()){
        memset(thruster_speeds, 0, sizeof(thruster_speeds));
    }
    for (uint8_t i = 0; i < NUM_THRUSTERS; ++i){
        thrusters[i].write(thruster_speeds[i]);
    }

	nh.spinOnce();

}
