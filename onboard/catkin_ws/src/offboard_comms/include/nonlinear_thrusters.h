#ifndef THRUSTERS_H
#define THRUSTERS_H

#include <custom_msgs/ThrusterAllocs.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include <array>

// Number of entries in each lookup table
const int NUM_LOOKUP_ENTRIES = 201;

// Lower and upper bounds of voltage
const double VOLTAGE_LOWER = 14.0;
const double VOLTAGE_UPPER = 18.0;

class Thrusters {
   private:
    double voltage;
    std::array<uint16_t, NUM_LOOKUP_ENTRIES> v14_lookup_table;
    std::array<uint16_t, NUM_LOOKUP_ENTRIES> v16_lookup_table;
    std::array<uint16_t, NUM_LOOKUP_ENTRIES> v18_lookup_table;

    ros::Subscriber thruster_allocs_sub;
    ros::Subscriber voltage_sub;

    ros::Publisher pwm_pub;

    void load_lookup_tables();
    void read_lookup_table_csv(const std::string &filename, std::array<uint16_t, NUM_LOOKUP_ENTRIES> &lookup_table);
    void voltage_callback(const std_msgs::Float64 &msg);
    void thruster_allocs_callback(const custom_msgs::ThrusterAllocs &msg);
    double lookup(double force);
    double interpolate(double x1, uint16_t y1, double x2, uint16_t y2, double x_interpolate);
    int round_to_two_decimals(double num);

   public:
    Thrusters(int argc, char **argv, ros::NodeHandle &nh);
};

#endif