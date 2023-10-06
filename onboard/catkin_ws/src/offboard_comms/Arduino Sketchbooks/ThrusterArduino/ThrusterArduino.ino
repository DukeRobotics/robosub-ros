#include "Adafruit_PWMServoDriver.h"
#include "MultiplexedServo.h"
#include "MultiplexedBasicESC.h"
#include <ros.h>
#include <custom_msgs/ThrusterSpeeds.h>
#include <custom_msgs/ServoAngleArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Bool.h>
#include <Arduino.h>

Adafruit_PWMServoDriver pwm_multiplexer(0x40);

#define BAUD_RATE 57600
#define NUM_THRUSTERS 8
#define THRUSTER_TIMEOUT_MS 500
#define NUM_SERVOS 8

uint64_t last_cmd_ms_ts;

int8_t thruster_speeds[NUM_THRUSTERS];
uint8_t servo_angles[NUM_SERVOS];

MultiplexedBasicESC thrusters[NUM_THRUSTERS];
MultiplexedServo servos[NUM_SERVOS];

// Sets node handle to have 3 subscribers, 2 publishers, and 128 bytes for input and output buffer
ros::NodeHandle_<ArduinoHardware,3,2,128,128> nh;

// Reusing ESC library code
void thruster_speeds_callback(const custom_msgs::ThrusterSpeeds &ts_msg){
    // Copy the contents of the speed message to the local array
    memcpy(thruster_speeds, ts_msg.speeds, sizeof(thruster_speeds));
    last_cmd_ms_ts = millis();
}

void servo_control_callback(const custom_msgs::ServoAngleArray &sa_msg){
    memcpy(servo_angles, sa_msg.angles, sizeof(servo_angles));
}

ros::Subscriber<custom_msgs::ThrusterSpeeds> ts_sub("/offboard/thruster_speeds", &thruster_speeds_callback);
ros::Subscriber<custom_msgs::ServoAngleArray> sa_sub("/offboard/servo_angles", &servo_control_callback);

void setup(){
    Serial.begin(BAUD_RATE);
    nh.getHardware()->setBaud(BAUD_RATE);
    nh.initNode();
    nh.subscribe(ts_sub);
    nh.subscribe(sa_sub);

    pwm_multiplexer.begin();
    for (uint8_t i = 0; i < NUM_THRUSTERS; ++i){
        thrusters[i].initialize(&pwm_multiplexer); 
        thrusters[i].attach(i);
    }
    for (uint8_t i = 0; i < NUM_SERVOS; ++i){
        servos[i].initialize(&pwm_multiplexer);
        servos[i].attach(i + NUM_THRUSTERS);
    }
    memset(servo_angles, 0, sizeof(servo_angles));
}

void loop(){
    // Check if last version of data has timed out, if so, reset the speeds
    if (last_cmd_ms_ts + THRUSTER_TIMEOUT_MS < millis()){
        memset(thruster_speeds, 0, sizeof(thruster_speeds));
    }
    for (uint8_t i = 0; i < NUM_THRUSTERS; ++i){
        thrusters[i].write(thruster_speeds[i]);
    }

    for(uint8_t i = 0; i < NUM_SERVOS; ++i){
        servos[i].write(servo_angles[i]);
    }

	nh.spinOnce();

}
