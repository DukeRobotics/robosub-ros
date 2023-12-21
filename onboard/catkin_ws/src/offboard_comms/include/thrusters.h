#ifndef THRUSTERS_H
#define THRUSTERS_H

#include <array>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <custom_msgs/ThrusterAllocs.h>

const int NUM_LOOKUP_ENTRIES = 201;

class Thrusters
{
public:
  double voltage;
  std::array<uint16_t, NUM_LOOKUP_ENTRIES> v14_lookup_table;
  std::array<uint16_t, NUM_LOOKUP_ENTRIES> v16_lookup_table;
  std::array<uint16_t, NUM_LOOKUP_ENTRIES> v18_lookup_table;

  ros::Subscriber thruster_allocs_sub;
  ros::Subscriber voltage_sub;

  ros::Publisher pwm_pub;

  Thrusters(int argc, char **argv, ros::NodeHandle &nh);
  double interpolate(double x1, uint16_t y1, double x2, uint16_t y2, double x_interpolate);
  double lookup(double force);
  void load_lookup_tables();
  void read_lookup_table_csv(const std::string &filename, std::array<uint16_t, NUM_LOOKUP_ENTRIES> &lookup_table);
  void voltage_callback(const std_msgs::Float32 &msg);
  void thruster_allocs_callback(const custom_msgs::ThrusterAllocs &msg);
};

#endif