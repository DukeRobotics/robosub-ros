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

Adafruit_PWMServoDriver pwm_multiplexer(0x40);
MS5837 sensor;

#define BAUD_RATE 57600
#define NUM_THRUSTERS 8
#define THRUSTER_TIMEOUT_MS 500
#define NUM_SERVOS 8

uint64_t last_cmd_ms_ts;

int8_t thruster_speeds[NUM_THRUSTERS];
uint8_t servo_angles[NUM_SERVOS];

MultiplexedBasicESC thrusters[NUM_THRUSTERS];
MultiplexedServo servos[NUM_SERVOS];

//Relay to hard reset camera
int relay = 2; //pin 2
bool camera_enabled = true;
uint64_t last_relay_msg;

// Sets node handle to have 3 subscribers, 1 publisher, and 128 bytes for input and output buffer
ros::NodeHandle_<ArduinoHardware,3,2,128,128> nh;

// 0 subscribers, 1 publisher, For depth sensor
ros::NodeHandle_<ArduinoHardware,1,1,1024,1024> nh2;

//Message to use with the relay status
std_msgs::Bool relay_status_msg;
geometry_msgs::PoseWithCovarianceStamped odom_msg;
ros::Publisher pressure_pub("/offboard/pressure", &odom_msg);
ros::Publisher relay_status_pub("/offboard/camera_relay_status", &relay_status_msg);

ros::Subscriber<custom_msgs::ThrusterSpeeds> ts_sub("/offboard/thruster_speeds", &thruster_speeds_callback);
ros::Subscriber<custom_msgs::ServoAngleArray> sa_sub("/offboard/servo_angles", &servo_control_callback);
ros::Subscriber<std_msgs::Bool> relay_sub("/offboard/camera_relay", &relay_callback);

// Reusing ESC library code
void thruster_speeds_callback(const custom_msgs::ThrusterSpeeds &ts_msg){
    // Copy the contents of the speed message to the local array
    memcpy(thruster_speeds, ts_msg.speeds, sizeof(thruster_speeds));
    last_cmd_ms_ts = millis();
}

void servo_control_callback(const custom_msgs::ServoAngleArray &sa_msg){
    memcpy(servo_angles, sa_msg.angles, sizeof(servo_angles));
}

void relay_callback(const std_msgs::Bool &relay_msg){
    //relay is normally closed, so turning it on means pulling the control pin low
    if (relay_msg.data) {
        digitalWrite(relay, LOW);
    }
    else {
        digitalWrite(relay, HIGH);
    }
    //log if change
    if (relay_msg.data != camera_enabled) {
        if (relay_msg.data) {
            nh.loginfo("Camera enabled");
        }
        else {
            nh.loginfo("Camera disabled");
            nh.logwarn("Do not leave camera diasbled for more than 3 minutes as it will draw excessive power to maintain the relay state");
        }
        camera_enabled = relay_msg.data;
        relay_status_msg.data = camera_enabled;
    }

}

void setup(){
    Serial.begin(BAUD_RATE);
    nh.getHardware()->setBaud(BAUD_RATE);
    nh.initNode();
    nh.subscribe(ts_sub);
    nh.subscribe(sa_sub);
    nh.subscribe(relay_sub);
    nh.advertise(relay_status_pub);

    Serial1.begin(9600)
    nh2.getHardware()->setBaud(9600);
    nh2.initNode();
    nh2.advertise(pressure_pub);

    Wire.begin();

    // Set up relay
    pinMode(relay, OUTPUT);
    digitalWrite(relay, LOW); //default to on (relay is normally closed)

    //sync with to camera_enabled on startup
    relay_status_msg.data = camera_enabled;

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

    while (!sensor.init()) {
        nh2.logwarn("Pressure sensor not initialized. Will attempt every 5 seconds until found.");
        delay(5000);
    }

    sensor.setModel(MS5837::MS5837_30BA);
    sensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)

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

    //publish relay if it has been at least a second since last publish
    if (millis() - last_relay_msg > 1000) {
        relay_status_pub.publish(&relay_status_msg);
        last_relay_msg = millis();
	    nh.spinOnce();
    }

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
    nh2.spinOnce();
}
