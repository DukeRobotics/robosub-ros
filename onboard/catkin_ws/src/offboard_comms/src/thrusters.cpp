#include <ros/ros.h>

#include "thrusters.h"

Thrusters::Thrusters(int argc, char **argv, ros::NodeHandle &nh)
{
  // Init
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "thrusters");
  ros::NodeHandle nh;

  Thrusters thrusters(argc, argv, nh);

  ros::spin();
  return 0;
}