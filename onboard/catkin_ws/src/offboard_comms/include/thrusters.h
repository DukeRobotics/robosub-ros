#ifndef THRUSTERS_H
#define THRUSTERS_H

#include <array>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <custom_msgs/ThrusterAllocs.h>

class Thrusters
{
public:
  double voltage;
  std::array<int16_t, 201> v14_lookup_table;
  std::array<int16_t, 201> v16_lookup_table;
  std::array<int16_t, 201> v18_lookup_table;

  ros::Subscriber thruster_allocs_sub;
  ros::Subscriber voltage_sub;

  ros::Publisher pwm_pub;

  Thrusters(int argc, char **argv, ros::NodeHandle &nh);
  double interpolate(float x1, float y1, float x2, float y2, float x_interpolate);
  double lookup(float voltage, float force);
  void load_lookup_tables();
  void read_lookup_table_csv(const std::string &filename, std::array<int16_t, 201> &lookup_table);
  void voltage_callback(const std_msgs::Float32 &msg);
  void thruster_allocs_callback(const custom_msgs::ThrusterAllocs &msg);
};

#endif