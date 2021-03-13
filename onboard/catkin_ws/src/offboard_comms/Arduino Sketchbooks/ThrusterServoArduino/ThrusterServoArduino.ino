#include "Adafruit_PWMServoDriver.h"
#include "MultiplexedServo.h"
#include "MultiplexedBasicESC.h"
#include "MS5837.h"
#include <ros.h>
#include <custom_msgs/ThrusterSpeeds.h>
#include <custom_msgs/ServoAngleArray.h>
#include <sensor_msgs/FluidPressure.h>
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

MS5837 pressure_sensor;

// Reusing ESC library code
void thruster_speeds_callback(const custom_msgs::ThrusterSpeeds &ts_msg){
    // Copy the contents of the speed message to the local array
    memcpy(thruster_speeds, ts_msg.speeds, sizeof(thruster_speeds));
    last_cmd_ms_ts = millis();
}

void servo_control_callback(const custom_msgs::ServoAngleArray &sa_msg){
    memcpy(servo_angles, sa_msg.angles, sizeof(servo_angles));
}

//Message to use with the pressure sensor
sensor_msgs::FluidPressure pressure_msg;

// Sets node handle to have 2 subscribers, 1 publishers, and 128 bytes for input and output buffer
ros::NodeHandle_<ArduinoHardware,2,1,128,128> nh;
ros::Subscriber<custom_msgs::ThrusterSpeeds> ts_sub("/offboard/thruster_speeds", &thruster_speeds_callback);
ros::Subscriber<custom_msgs::ServoAngleArray> sa_sub("/offboard/servo_angles", &servo_control_callback);
ros::Publisher pressure_pub("/offboard/pressure", &pressure_msg);

void setup(){
    Serial.begin(BAUD_RATE);
    nh.initNode();
    nh.subscribe(ts_sub);
    nh.subscribe(sa_sub);
    nh.advertise(pressure_pub);

    pwm_multiplexer.begin();
    for (uint8_t i = 0; i < NUM_THRUSTERS; ++i){
        thrusters[i].initialize(&pwm_multiplexer);
        thrusters[i].attach(i);
    }
    for (uint8_t i = 0; i < NUM_SERVOS; ++i){
        servos[i].initialize(&pwm_multiplexer);
        servos[i].attach(i + NUM_THRUSTERS);
    }

    Wire.begin();
    while(!pressure_sensor.init()){
      nh.logerror("Failed to initialize pressure sensor.");
      delay(2000);
    }
    pressure_sensor.setModel(MS5837::MS5837_30BA);

    // Wait for motors to fully initialise
    delay(2000);
    last_cmd_ms_ts = millis();
}

void loop(){
    // Check if last version of data has timed out, if so, reset the speeds
    if (last_cmd_ms_ts + THRUSTER_TIMEOUT_MS < millis())
        memset(thruster_speeds, 0, sizeof(thruster_speeds));
    for (uint8_t i = 0; i < NUM_THRUSTERS; ++i){
        thrusters[i].write(thruster_speeds[i]);
    }
    for(uint8_t i = 0; i < NUM_SERVOS; ++i){
        servos[i].write(servo_angles[i]);
    }
    pressure_sensor.read();
    pressure_msg.fluid_pressure = pressure_sensor.pressure(100.0f);
    pressure_pub.publish(&pressure_msg);
    nh.spinOnce();
}
