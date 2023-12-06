#include <ros/ros.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <std_msgs/String.h>
#include "yaml-cpp/yaml.h"

int main(int argc, char** argv)
{
  // tf2::Vector3 v_orig(0, 0, -1) ;
  // tf2::Quaternion q( 0, 0.7071068, 0, 0.7071068 ) ;
  // tf2::Vector3 v_new = quatRotate(q.inverse(), v_orig);
  // std::cout << "v_orig: " << v_orig.getX() << ", " << v_orig.getY() << ", " << v_orig.getZ() << std::endl;
  // std::cout << "q: " << q.getX() << ", " << q.getY() << ", " << q.getZ() << ", " << q.getW() << std::endl;
  // std::cout << "v_new: " << v_new.getX() << ", " << v_new.getY() << ", " << v_new.getZ() << std::endl;
  // return 0;

  // Get rpy from quaternion
  tf2::Quaternion q(0, 0, 0.9999912, -0.0042037);
  tf2::Quaternion nq = q.normalized();
  std::cout << "q: " << q.getX() << ", " << q.getY() << ", " << q.getZ() << ", " << q.getW() << std::endl;
  std::cout << "nq: " << nq.getX() << ", " << nq.getY() << ", " << nq.getZ() << ", " << nq.getW() << std::endl;

  std::cout << std::endl;

  // Equivalent to ZYX and python's quaternion_from_euler
  // In other words, given a set of roll, pitch, yaw angles (in radians) converted into a quaternion with
  // quaternion_from_euler with the default value for its axes arg (sxyz), the code below (getRPY), called on that
  // quaternion, will return the same original roll, pitch, yaw angles (in radians).
  double roll, pitch, yaw;
  tf2::Matrix3x3(nq).getRPY(roll, pitch, yaw);
  std::cout << "roll: " << roll << std::endl;
  std::cout << "pitch: " << pitch << std::endl;
  std::cout << "yaw: " << yaw << std::endl;

  std::cout << std::endl;

  //
  tf2::Matrix3x3(nq).getEulerYPR(roll, pitch, yaw);
  std::cout << "roll: " << roll << std::endl;
  std::cout << "pitch: " << pitch << std::endl;
  std::cout << "yaw: " << yaw << std::endl;

  return 0;

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