#include "Adafruit_PWMServoDriver.h"
#include "MultiplexedBasicESC.h"
#include "MultiplexedServo.h"
#include "MS5837.h"
#include <ros.h>
#include <custom_msgs/ThrusterSpeeds.h>
#include <custom_msgs/ServoAngle.h>
#include <sensor_msgs/FluidPressure.h>
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
MS5837 pressure_sensor;

// Reusing ESC library code
void thruster_speeds_callback(const custom_msgs::ThrusterSpeeds &ts_msg){
    // Copy the contents of the speed message to the local array
    memcpy(thruster_speeds, ts_msg.speeds, sizeof(thruster_speeds));
    last_cmd_ms_ts = millis();
}

void servo_control_callback(const custom_msgs::ServoAngle &sa_msg){
    if(sa_msg.num >= NUM_SERVO || sa_msg.angle > 180){
        return;
    }
    servos[sa_msg.num]->run(sa_msg.angle);
}

//Message to use with the pressure sensor
sensor_msgs::FluidPressure pressure_msg;

// Sets node handle to have 2 subscribers, 0 publishers, and 150 bytes for input and output buffer
ros::NodeHandle_<ArduinoHardware,2,2,150,150> nh;
ros::Subscriber<custom_msgs::ThrusterSpeeds> ts_sub("/offboard/thruster_speeds", &thruster_speeds_callback);
ros::Subscriber<custom_msgs::ServoAngle> sa_sub("/offboard/servo_angles", &servo_control_callback);
ros::Publisher pressure_pub("/offboard/pressure", &pressure_msg);

void setup(){
    Serial.begin(BAUD_RATE);
    nh.initNode();
    nh.subscribe(ts_sub);
    nh.subscribe(sa_sub);
    nh.advertise(pressure_pub);

    pwm_multiplexer.begin();
    for (uint8_t i = 0; i < NUM_THRUSTERS; ++i){
        thrusters[i] = new MultiplexedBasicESC(&pwm_multiplexer, i);
        thrusters[i]->initialise();
    }
    for (uint8_t i = 0; i < NUM_SERVO; ++i){
        servos[i] = new MultiplexedServo(&pwm_multiplexer, i + NUM_THRUSTERS);
        servos[i]->initialise();
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
        thrusters[i]->run(thruster_speeds[i]);
    }
    pressure_sensor.read();
    pressure_msg.fluid_pressure = pressure_sensor.pressure(100.0f);
    pressure_pub.publish(&pressure_msg);
    nh.spinOnce();
}
