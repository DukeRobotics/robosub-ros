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
  // Set a default voltage
  voltage = 15.5;
  // Read in corresponding voltage tables for 14.0, 16.0, and 18.0v
  load_lookup_tables();

  // Instantiate a subscriber to the voltage sensor, should read voltages from range \in [14.0, 18.0]
  voltage_sub = nh.subscribe("/sensors/voltage", 1, &Thrusters::voltage_callback, this);
  // Subscribe to thruster allocation (desired range \in [-1.0, 1.0])
  thruster_allocs_sub = nh.subscribe("/controls/thruster_allocs", 1, &Thrusters::thruster_allocs_callback, this);
  // Publish pwm allocation based on calculated conversions
  pwm_pub = nh.advertise<custom_msgs::PWMAllocs>("/offboard/pwm", 1);
}

// This method reads in the csv files for 14, 16, 18v volt conversions
// Each lookup table stores a thruster allocation from -1.0 to 1.0
// with voltage \in 14.0 to 18.0v and the appropriate pwm output
void Thrusters::load_lookup_tables()
{
  std::string package_path = ros::package::getPath("offboard_comms");

  read_lookup_table_csv(package_path + "/data/14.csv", v14_lookup_table);
  read_lookup_table_csv(package_path + "/data/16.csv", v16_lookup_table);
  read_lookup_table_csv(package_path + "/data/18.csv", v18_lookup_table);
}

// Read lookup table for voltage, thruster alloc, and pwm alloc; thurster alloc/force is stored under 2 decimal precision
void Thrusters::read_lookup_table_csv(const std::string &filename, std::array<int16_t, NUM_LOOKUP_ENTRIES> &lookup_table)
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

// Update the voltage field; used in pwn interpolation
void Thrusters::voltage_callback(const std_msgs::Float32 &msg)
{
  voltage = msg.data;
}

// Given thruster allocation, we reference our last read voltage
// to calculate the appropriate pwm allocation based on voltage and thruster alloc
void Thrusters::thruster_allocs_callback(const custom_msgs::ThrusterAllocs &msg)
{
  // Set timestamp to now; set up PWMAlloc object to publish
  custom_msgs::PWMAllocs pwm_msg;
  pwm_msg.header.stamp = ros::Time::now();
  pwm_msg.allocs.resize(msg.allocs.size());

  // For each of the 8 thrusters in thruster alloc (all of values in -1.0, 1.0)
  // perform a linear interpolation based on the nearest lookup table entries
  for (int i = 0; i < msg.allocs.size(); i++)
    pwm_msg.allocs[i] = lookup(msg.allocs[i]);

  // Publish interpolated values for each thruster
  pwm_pub.publish(pwm_msg);
}

// Given thruster alloc (force) and the current voltage stored as a field, compute which lookup tables
// to use to perform linear interpolation
double Thrusters::lookup(double force)
{
  // Check to make sure force and voltage are not out of bounds
  ROS_ASSERT_MSG(-1.0 <= force && force <= 1.0, "Force must be in [-1.0, 1.0]");
  ROS_ASSERT_MSG(14.0 <= voltage && voltage <= 18.0, "Voltage must be in [14.0, 18.0]");

  // Round force to 2 decimal points (which is what is used to index the lookup tables)
  int index = static_cast<int>(std::lround((force * 100) + 100));
  ROS_ASSERT_MSG(0 <= index && index < NUM_LOOKUP_ENTRIES, "Error in PWM lookup. Index %d is out of bounds.", index);

  if (14.0 <= voltage && voltage <= 16.0)
    // Rounded force is between 14.0 and 16.0, use those lookup tables to interpolate
    return interpolate(14.0, v14_lookup_table.at(index), 16.0, v16_lookup_table.at(index), voltage);
  else
    // Rounded force is between 16.0 and 18.0, use those lookup tables to interpolate
    return interpolate(16.0, v16_lookup_table.at(index), 18.0, v18_lookup_table.at(index), voltage);
}

// Perform linear interpolation to compute PWM given force and voltage
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