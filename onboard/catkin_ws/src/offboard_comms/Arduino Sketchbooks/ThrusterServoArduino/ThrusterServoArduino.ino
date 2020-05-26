#include "Adafruit_PWMServoDriver.h"
#include "MultiplexedBasicESC.h"
#include "MultiplexedServo.h"
#include <ros.h>
#include <cthulhu_msgs/ThrusterSpeeds.h>
#include <cthulhu_msgs/SetServo.h>
#include <Arduino.h>

Adafruit_PWMServoDriver pwm_multiplexer(0x40);

#define BAUD_RATE 57600
#define NUM_THRUSTERS 8
#define THRUSTER_TIMEOUT_MS 500
#define NUM_SERVO 8

uint64_t last_cmd_ms_ts;

int8_t thruster_speeds[NUM_THRUSTERS];

MultiplexedBasicESC *thrusters[NUM_THRUSTERS];
MultiplexedServo *servos[NUM_SERVO];

// reusing ESC library code
void thruster_speeds_callback(const offboard_comms::ThrusterSpeeds &ts_msg){
    //copy the contents of the speed message to the local array
    memcpy(thruster_speeds, ts_msg.speeds, sizeof(thruster_speeds));
    last_cmd_ms_ts = millis();
}

void servo_control_callback(const offboard_comms::SetServo::Request &sc_req, offboard_comms::SetServo::Response &sc_res){
    //copy the contents of the servo request to the local variables
    uint8_t pin = sc_req.num;
    uint16_t angle = sc_req.angle;
    if(pin >= NUM_SERVO || angle > 180){
      sc_res.success = false;
      return;
    }
    servos[pin]->run(angle);
    sc_res.success = true;
}

//Sets node handle to have 2 subscribers, 2 publishers, and 150 bytes for input and output buffer
ros::NodeHandle_<ArduinoHardware,2,2,150,150> nh;  
ros::Subscriber<offboard_comms::ThrusterSpeeds> ts_sub("/offboard/thruster_speeds", &thruster_speeds_callback);
ros::ServiceServer<offboard_comms::SetServo::Request, offboard_comms::SetServo::Response> servo_service("/offboard/servo_angle", &servo_control_callback);

void setup(){
    Serial.begin(BAUD_RATE);
    nh.initNode();
    nh.subscribe(ts_sub);
    nh.advertiseService(servo_service);
    pwm_multiplexer.begin();
    for (int i = 0; i < NUM_THRUSTERS; ++i){
        thrusters[i] = new MultiplexedBasicESC(&pwm_multiplexer, i);
        thrusters[i]->initialise();
    }
    for (int i = 0; i < NUM_SERVO; ++i){
        servos[i] = new MultiplexedServo(&pwm_multiplexer, i + NUM_THRUSTERS);
        servos[i]->initialise();
    }
    // Wait for motors to fully initialise
    delay(2000);
    last_cmd_ms_ts = millis();
}

void loop(){
    // check if last version of data has timed out, if so, reset the speeds
    if (last_cmd_ms_ts + THRUSTER_TIMEOUT_MS < millis())
        memset(thruster_speeds, 0, sizeof(thruster_speeds));
    for (int i = 0; i < NUM_THRUSTERS; ++i){
        thrusters[i]->run(thruster_speeds[i]);
    }
    nh.spinOnce();
}
