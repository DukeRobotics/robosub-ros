#include <ros/ros.h>
#include <std_msgs/String.h>
#include "yaml-cpp/yaml.h"

int main(int argc, char** argv)
{
  YAML::Emitter out;
  out << "Hello, World!";

  std::cout << "Here's the output YAML:\n" << out.c_str(); // prints "Hello, World!"
  return 0;

  // Initialize the ROS node
  ros::init(argc, argv, "hello_world_publisher");

  // Create a ROS node handle
  ros::NodeHandle nh;

  // Create a publisher for the "hello_world" topic
  ros::Publisher pub = nh.advertise<std_msgs::String>("hello_world_cpp", 10);

  // Set the publishing rate (in Hz)
  ros::Rate rate(50000);  // Publish at 1 Hz (1 message per second)

  while (ros::ok())
  {
    // Create a message and set its data
    std_msgs::String msg;
    msg.data = "Hello, World!";

    // Publish the message
    pub.publish(msg);

    // Spin once to process callbacks
    ros::spinOnce();

    // Sleep to maintain the desired publishing rate
    rate.sleep();
  }

  return 0;
}