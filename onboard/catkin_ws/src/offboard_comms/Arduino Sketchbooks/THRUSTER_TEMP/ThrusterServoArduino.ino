#include <ros.h>
#include <sensor_msgs/FluidPressure.h>
#include <Wire.h>
#include "MS5837.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>

MS5837 sensor;

// 0 subscribers, 1 publisher
ros::NodeHandle_<ArduinoHardware,1,1,1024,1024> nh;

// Message for fluid pressure
// sensor_msgs::FluidPressure pressure_msg;
geometry_msgs::PoseWithCovarianceStamped odom_msg;

ros::Publisher pressure_pub("/offboard/pressure", &odom_msg);

void setup() {
  
  Serial.begin(9600);
  nh.getHardware()->setBaud(9600);
  nh.initNode();
  nh.advertise(pressure_pub);
   
  Wire.begin();

  // Initialize pressure sensor
  // Returns true if initialization was successful
  // We can't continue with the rest of the program unless we can initialize the sensor
  while (!sensor.init()) {
    nh.logwarn("Pressure sensor not initialized. Will attempt every 5 seconds until found.");
    delay(5000);
  }
  
  sensor.setModel(MS5837::MS5837_30BA);
  sensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)
}

void loop() {
  // Update pressure and temperature readings
  sensor.read();
  char c[50];
  sprintf(c, "%g", sensor.pressure(100.0f));
  // nh.loginfo(c);

  //odom_msg.header.stamp = ros::Time::now();
  odom_msg.header.frame_id = "odom";

  geometry_msgs::Point point;
  point.x = 0.0;
  point.y = 0.0;
  point.z = sensor.depth();
  // point.z = sensor.altitude();

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

  //pressure_msg.fluid_pressure = sensor.pressure(100.0f);

  pressure_pub.publish(&odom_msg);

  nh.spinOnce();

  delay(100);

}
