#include <ros.h>
#include <Arduino.h>
#include <Wire.h>
#include "MS5837.h"
#include <std_msgs/Float64.h>

MS5837 sensor;

// Baud rate for serial communication with Blue Robotics Bar30 High-Resolution 300m Depth/Pressure Sensor
#define BAUD_RATE 9600

// 1 publisher for depth sensor
ros::NodeHandle_<ArduinoHardware,1,1,128,128> nh;

std_msgs::Float64 depth;
ros::Publisher pressure_pub("/offboard/pressure", &depth);

void setup(){

    Serial1.begin(BAUD_RATE);
    nh.getHardware()->setBaud(BAUD_RATE);
    nh.initNode();
    nh.advertise(pressure_pub);

    Wire.begin();

    while (!sensor.init()) {
        nh.logwarn("Pressure sensor not initialized. Will attempt every second until found.");
        delay(1000);
        break;
    }

    sensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)
}

void loop(){

    sensor.read();

    depth = sensor.depth()

    pressure_pub.publish(&depth);

    nh.spinOnce();

    // odom_msg.header.frame_id = "odom";

    // // Set all values to 0 except for depth
    // // Zeroed values are not used by the EKF
    // geometry_msgs::Point point;
    // point.x = 0.0;
    // point.y = 0.0;
    // point.z = sensor.depth();

    // // Unit quaternion -- also unused by EKF
    // geometry_msgs::Quaternion quat;
    // quat.x = 0.0;
    // quat.y = 0.0;
    // quat.z = 0.0;
    // quat.w = 1.0;

    // geometry_msgs::Pose pose;
    // pose.position = point;
    // pose.orientation = quat;
    // odom_msg.pose.pose = pose;
    // odom_msg.pose.covariance[14] = 0.01; // depth variance

    // pressure_pub.publish(&odom_msg);
}
