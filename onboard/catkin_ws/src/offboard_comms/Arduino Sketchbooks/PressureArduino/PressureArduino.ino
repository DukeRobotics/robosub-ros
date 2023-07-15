#include "Adafruit_PWMServoDriver.h"
#include "MultiplexedServo.h"
#include "MultiplexedBasicESC.h"
#include <ros.h>
#include <custom_msgs/ThrusterSpeeds.h>
#include <custom_msgs/ServoAngleArray.h>
#include <sensor_msgs/FluidPressure.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Bool.h>
#include <Arduino.h>
#include <Wire.h>
#include "MS5837.h"
#include <SoftwareSerial.h>

MS5837 sensor;

#define BAUD_RATE 9600

// 0 subscribers, 1 publisher, For depth sensor
ros::NodeHandle_<ArduinoHardware,1,1,1024,1024> nh;

geometry_msgs::PoseWithCovarianceStamped odom_msg;
ros::Publisher pressure_pub("/offboard/pressure", &odom_msg);


void setup(){

    Serial1.begin(BAUD_RATE);
    nh.getHardware()->setBaud(BAUD_RATE);
    nh.initNode();
    nh.advertise(pressure_pub);

    Wire.begin();

    while (!sensor.init()) {
        nh.logwarn("Pressure sensor not initialized. Will attempt every 5 seconds until found.");
        delay(5000);
        break;
    }

    sensor.setModel(MS5837::MS5837_30BA);
    sensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)

}

void loop(){

    sensor.read();

    odom_msg.header.frame_id = "odom";

    geometry_msgs::Point point;
    point.x = 0.0;
    point.y = 0.0;
    point.z = sensor.depth();

    geometry_msgs::Quaternion quat;
    quat.x = 0.0;
    quat.y = 0.0;
    quat.z = 0.0;
    quat.w = 0.0;

    geometry_msgs::Pose pose;
    pose.position = point;
    pose.orientation = quat;
    odom_msg.pose.pose = pose;
    odom_msg.pose.covariance[14] = 0.01; // depth variance

    pressure_pub.publish(&odom_msg);
    nh.spinOnce();
}


// #include "Adafruit_PWMServoDriver.h"
// #include "MultiplexedServo.h"
// #include "MultiplexedBasicESC.h"
// #include <ros.h>
// #include <custom_msgs/ThrusterSpeeds.h>
// #include <custom_msgs/ServoAngleArray.h>
// #include <sensor_msgs/FluidPressure.h>
// #include <geometry_msgs/PoseWithCovarianceStamped.h>
// #include <std_msgs/Bool.h>
// #include <Arduino.h>

// Adafruit_PWMServoDriver pwm_multiplexer(0x40);

// #define BAUD_RATE 57600
// #define NUM_THRUSTERS 8
// #define THRUSTER_TIMEOUT_MS 500
// #define NUM_SERVOS 8

// uint64_t last_cmd_ms_ts;

// int8_t thruster_speeds[NUM_THRUSTERS];
// uint8_t servo_angles[NUM_SERVOS];

// MultiplexedBasicESC thrusters[NUM_THRUSTERS];
// MultiplexedServo servos[NUM_SERVOS];

// bool has_pressure = true;

// //Relay to hard reset camera
// int relay = 2; //pin 2
// bool camera_enabled = true;
// uint64_t last_relay_msg;

// // Sets node handle to have 1 publisher, and 128 bytes for input and output buffer
// ros::NodeHandle_<ArduinoHardware,1,1,128,128> nh;

// geometry_msgs::PoseWithCovarianceStamped depth;


// void setup(){
//     Serial.begin(BAUD_RATE);
//     nh.getHardware()->setBaud(BAUD_RATE);
//     nh.initNode();
//     nh.subscribe(ts_sub);
//     nh.subscribe(sa_sub);
//     nh.subscribe(relay_sub);
//     nh.advertise(relay_status_pub);


//     //sync with to camera_enabled on startup
//     relay_status_msg.data = camera_enabled;

//     pwm_multiplexer.begin();
//     for (uint8_t i = 0; i < NUM_THRUSTERS; ++i){
//         thrusters[i].initialize(&pwm_multiplexer); 
//         thrusters[i].attach(i);
//     }
//     for (uint8_t i = 0; i < NUM_SERVOS; ++i){
//         servos[i].initialize(&pwm_multiplexer);
//         servos[i].attach(i + NUM_THRUSTERS);
//     }
//     memset(servo_angles, 0, sizeof(servo_angles));

// //     Wire.setClock(400*1000);
// //    Wire.begin();
// //     #if defined(WIRE_HAS_TIMEOUT)
// //         Wire.setWireTimeout(3000, true);
// //     #endif
//     int pressure_attempts = 0;
//     while(!pressure_sensor.init()){
//       nh.logerror("Failed to initialize pressure sensor.");
//       if(++pressure_attempts > 3){
//          has_pressure = false;
//          break;
//       }
//       delay(2000);
//     }
//     if(has_pressure) pressure_sensor.setModel(MS5837::MS5837_30BA);

//     // Wait for motors to fully initialise
//     delay(2000);
//     last_cmd_ms_ts = millis();
//     last_relay_msg = millis();
//     // last_pressure_msg = millis(); 
// }

// void loop(){
//     // Check if last version of data has timed out, if so, reset the speeds
//     if (last_cmd_ms_ts + THRUSTER_TIMEOUT_MS < millis()){
//         memset(thruster_speeds, 0, sizeof(thruster_speeds));
//     }
//     for (uint8_t i = 0; i < NUM_THRUSTERS; ++i){
//         thrusters[i].write(thruster_speeds[i]);
//     }

//     // for(uint8_t i = 0; i < NUM_SERVOS; ++i){
//     //     servos[i].write(servo_angles[i]);
//     // }

//     //publish relay if it has been at least a second since last publish
//     if (millis() - last_relay_msg > 1000) {
//         relay_status_pub.publish(&relay_status_msg);
//         last_relay_msg = millis();
//     }

// 	nh.spinOnce();

// }
