#include <fstream>
#include <sstream>
#include <vector>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/assert.h>
#include <std_msgs/Float64.h>
#include <custom_msgs/PWMAllocs.h>

#include "thrusters.h"

Thrusters::Thrusters(int argc, char **argv, ros::NodeHandle &nh)
{
  // Set the default voltage
  voltage = 15.5;

  // Read in corresponding voltage tables for 14.0, 16.0, and 18.0V
  load_lookup_tables();

  // Instantiate a subscriber to the voltage sensor, should read voltages from range in [14.0, 18.0]
  voltage_sub = nh.subscribe("/sensors/voltage", 1, &Thrusters::voltage_callback, this);

  // Subscribe to thruster allocation (desired range in [-1.0, 1.0])
  thruster_allocs_sub = nh.subscribe("/controls/thruster_allocs", 1, &Thrusters::thruster_allocs_callback, this);

  // Publish pwm allocation based on calculated conversions
  pwm_pub = nh.advertise<custom_msgs::PWMAllocs>("/offboard/pwm", 1);
}

// This method reads in the csv files for 14.0, 16.0, 18.0V volt conversions
// Each lookup table stores all thruster allocations from -1.0 to 1.0 in 0.01 increments
// with voltage in 14.0 to 18.0V and the appropriate pwm output
void Thrusters::load_lookup_tables()
{
  std::string package_path = ros::package::getPath("offboard_comms");

  read_lookup_table_csv(package_path + "/data/14.csv", v14_lookup_table);
  read_lookup_table_csv(package_path + "/data/16.csv", v16_lookup_table);
  read_lookup_table_csv(package_path + "/data/18.csv", v18_lookup_table);
}

// Read lookup table for voltage, thruster alloc, and pwm alloc; thurster alloc/force is stored with 2 decimal precision
void Thrusters::read_lookup_table_csv(const std::string &filename, std::array<uint16_t, NUM_LOOKUP_ENTRIES> &lookup_table)
{
  std::ifstream file(filename);
  ROS_ASSERT_MSG(file.is_open(), "Error opening file %s", filename.c_str());

  std::string line, col;

  // Read header line (assuming headers exist and need to be skipped)
  std::getline(file, line);

  // Read each line of the file
  while (std::getline(file, line))
  {
    std::istringstream ss(line);
    std::vector<std::string> row;

    // Read each column of the line and store each column in the row vector
    while (std::getline(ss, col, ','))
    {
      row.push_back(col);
    }

    if (row.size() >= 2)
    {
      // Assuming PWM is in the first column and force is in the second column
      double force = std::stod(row[0]);
      int pwm = std::stoi(row[1]);

      // If the PWM value is within bounds of uint16_t, cast it to uint16_t
      uint16_t pwm16;
      ROS_ASSERT_MSG(pwm <= static_cast<int>(UINT16_MAX) && pwm >= 0, "Error while reading CSV. PWM value %d is out of bounds for uint16_t.", pwm);
      pwm16 = static_cast<uint16_t>(pwm);

      // Multiply the force value by 100 and add 100
      // This is done to convert the force value to an index for the lookup table
      // It is important to use std::lround() instead of type casting to int directly because a direct typecast
      // can result in the incorrect index being calculated (an error of +/- 1)
      int index = round_to_two_decimals(force);

      // If the index is within bounds of the array, store the pwm value at that index
      ROS_ASSERT_MSG(index >= 0 && index < lookup_table.size(), "Error while reading CSV. Index %d is out of bounds.", index);
      lookup_table[index] = pwm16;
    }
  }
  file.close();
}

// Update the voltage field; used in pwm interpolation
void Thrusters::voltage_callback(const std_msgs::Float64 &msg)
{
  // Clamp voltage to be within bounds
  voltage = std::max(VOLTAGE_LOWER, std::min(msg.data, VOLTAGE_UPPER));

  if (voltage != msg.data)
    ROS_WARN("Voltage %.2f out of bounds for thrust allocation to PWM conversion. Clamping to [%.2f, %.2f].", msg.data, VOLTAGE_LOWER, VOLTAGE_UPPER);
}

// Given thruster allocation, we reference our last received voltage
// to calculate the appropriate pwm allocation based on voltage and thruster alloc
void Thrusters::thruster_allocs_callback(const custom_msgs::ThrusterAllocs &msg)
{
  // Set timestamp to now; set up PWMAllocs object to publish
  custom_msgs::PWMAllocs pwm_msg;
  pwm_msg.header.stamp = ros::Time::now();

  // For each of the 8 thrusters in thruster alloc (all of values in -1.0, 1.0)
  // perform a linear interpolation based on the nearest lookup table entries
  for (int i = 0; i < msg.allocs.size(); i++)
    pwm_msg.allocs.push_back(lookup(msg.allocs[i]));

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
  int index = round_to_two_decimals(force);
  ROS_ASSERT_MSG(0 <= index && index < NUM_LOOKUP_ENTRIES, "Error in PWM lookup. Index %d is out of bounds.", index);

  if (14.0 <= voltage && voltage <= 16.0)
    // Rounded force is between 14.0 and 16.0, use those lookup tables to interpolate
    return interpolate(14.0, v14_lookup_table.at(index), 16.0, v16_lookup_table.at(index), voltage);
  else
    // Rounded force is between 16.0 and 18.0, use those lookup tables to interpolate
    return interpolate(16.0, v16_lookup_table.at(index), 18.0, v18_lookup_table.at(index), voltage);
}

// Perform linear interpolation to compute PWM given force and voltage
double Thrusters::interpolate(double x1, uint16_t y1, double x2, uint16_t y2, double x_interpolate)
{
  return y1 + ((y2 - y1) * (x_interpolate - x1)) / (x2 - x1);
}

// Round to two decimal places
int Thrusters::round_to_two_decimals(double num)
{
  return static_cast<int>(std::lround((num * 100) + 100));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "thrusters");
  ros::NodeHandle nh;

  Thrusters thrusters(argc, argv, nh);

  ros::spin();
  return 0;
}