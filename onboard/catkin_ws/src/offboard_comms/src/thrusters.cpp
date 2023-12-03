#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/assert.h>
#include <custom_msgs/PWMAllocs.h>

#include "thrusters.h"

Thrusters::Thrusters(int argc, char **argv, ros::NodeHandle &nh)
{
  voltage = 15.5;
  load_lookup_tables();

  voltage_sub = nh.subscribe("/sensors/voltage", 1, &Thrusters::voltage_callback, this);
  thruster_allocs_sub = nh.subscribe("/controls/thruster_allocs", 1, &Thrusters::thruster_allocs_callback, this);

  pwm_pub = nh.advertise<custom_msgs::PWMAllocs>("/offboard/pwm", 1);
}

void Thrusters::load_lookup_tables()
{
  std::string package_path = ros::package::getPath("offboard_comms");

  read_lookup_table_csv(package_path + "/data/14.csv", v14_lookup_table);
  read_lookup_table_csv(package_path + "/data/16.csv", v16_lookup_table);
  read_lookup_table_csv(package_path + "/data/18.csv", v18_lookup_table);
}

void Thrusters::read_lookup_table_csv(const std::string &filename, std::array<int16_t, 201> &lookup_table)
{
  std::ifstream file(filename);
  ROS_ASSERT_MSG(file.is_open(), "Error opening file %s", filename.c_str());

  std::string line, col;
  std::getline(file, line); // Read header line (assuming headers exist and need to be skipped)

  while (std::getline(file, line))
  {
    std::istringstream ss(line);
    std::vector<std::string> row;

    while (std::getline(ss, col, ','))
    {
      row.push_back(col);
    }

    if (row.size() >= 2)
    {
      // Assuming PWM is in the first column and force is in the second column
      double force = std::stod(row[0]);
      double pwm = std::stod(row[1]);

      // Multiply the force value by 100 and add 100
      // This is done to convert the force value to an index for the lookup table
      // It is important to use std::lround() instead of type casting to int directly because a direct typecast
      // can result in the incorrect index being calculated (an error of +/- 1)
      int index = static_cast<int>(std::lround((force * 100) + 100));

      // Check if the index is within bounds of the array
      if (index >= 0 && index < lookup_table.size())
        lookup_table[index] = pwm;
      else
        ROS_ASSERT_MSG(false, "Error while reading CSV. Index %d is out of bounds.", index);
    }
  }
  file.close();
}

void Thrusters::voltage_callback(const std_msgs::Float32 &msg)
{
  voltage = msg.data;
}

void Thrusters::thruster_allocs_callback(const custom_msgs::ThrusterAllocs &msg)
{
  custom_msgs::PWMAllocs pwm_msg;
  pwm_msg.header.stamp = ros::Time::now();

  pwm_msg.allocs.resize(msg.allocs.size());

  for (int i = 0; i < msg.allocs.size(); i++)
    pwm_msg.allocs[i] = lookup(voltage, msg.allocs[i]);

  pwm_pub.publish(pwm_msg);
}

double Thrusters::lookup(float voltage, float force)
{
  ROS_ASSERT_MSG(-1.0 <= force && force <= 1.0, "Force must be in [-1.0, 1.0]");
  ROS_ASSERT_MSG(14.0 <= voltage && voltage <= 18.0, "Voltage must be in [14.0, 18.0]");

  float rounded_force = roundf(force * 100) / 100;
  int index = (int)((rounded_force + 1) * 100);
  if (14.0 <= voltage && voltage <= 16.0)
    return interpolate(14.0, v14_lookup_table.at(index), 16.0, v16_lookup_table.at(index), voltage);
  else
    return interpolate(16.0, v16_lookup_table.at(index), 18.0, v18_lookup_table.at(index), voltage);
}

double Thrusters::interpolate(double x1, int16_t y1, double x2, int16_t y2, double x_interpolate)
{
  return y1 + ((y2 - y1) * (x_interpolate - x1)) / (x2 - x1);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "thrusters");
  ros::NodeHandle nh;

  Thrusters thrusters(argc, argv, nh);

  ros::spin();
  return 0;
}