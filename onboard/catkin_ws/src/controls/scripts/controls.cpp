#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <custom_msgs/ThrusterAllocs.h>
#include <custom_msgs/PIDGain.h>
#include <custom_msgs/PIDGains.h>
#include <custom_msgs/ControlTypes.h>
#include <custom_msgs/SetPIDGains.h>
#include <custom_msgs/SetControlTypes.h>

class Controls
{
public:
  static const int THRUSTER_ALLOCS_RATE = 20;

  bool debug;
  bool enable_position_pid;
  bool enable_velocity_pid;

  ros::Subscriber desired_position_sub;
  ros::Subscriber desired_velocity_sub;
  ros::Subscriber desired_power_sub;
  ros::Subscriber state_sub;

  geometry_msgs::Pose desired_position;
  geometry_msgs::Twist desired_velocity;
  geometry_msgs::Twist desired_power;
  nav_msgs::Odometry state;

  ros::ServiceServer enable_controls_srv;
  ros::ServiceServer set_control_types_srv;
  ros::ServiceServer set_pid_gains_srv;

  Controls(int argc, char **argv)
  {
    // Initialize ROS node
    ros::init(argc, argv, "controls");

    // Create a ROS node handle
    ros::NodeHandle nh;

    // Get parameters from launch file
    nh.param<bool>("debug", debug, false);
    nh.param<bool>("enable_position_pid", enable_position_pid, false);
    nh.param<bool>("enable_velocity_pid", enable_velocity_pid, false);

    // Subscribe to input topics
    desired_position_sub = nh.subscribe("controls/desired_position", 1, &Controls::desired_position_callback, this);
    desired_velocity_sub = nh.subscribe("controls/desired_velocity", 1, &Controls::desired_velocity_callback, this);
    desired_power_sub = nh.subscribe("controls/desired_power", 1, &Controls::desired_power_callback, this);
    state_sub = nh.subscribe("state", 1, &Controls::state_callback, this);

    // Advertise input services
    enable_controls_srv = nh.advertiseService("controls/enable", &Controls::enable_controls_callback, this);
    set_control_types_srv = nh.advertiseService("controls/set_control_types", &Controls::set_control_types_callback, this);
    set_pid_gains_srv = nh.advertiseService("controls/set_pid_gains", &Controls::set_pid_gains_callback, this);
  }

  void desired_position_callback(const geometry_msgs::Pose msg)
  {
    desired_position = msg;
  }

  void desired_velocity_callback(const geometry_msgs::Twist msg)
  {
    desired_velocity = msg;
  }

  void desired_power_callback(const geometry_msgs::Twist msg)
  {
    desired_power = msg;
  }

  void state_callback(const nav_msgs::Odometry msg)
  {
    state = msg;
  }

  bool enable_controls_callback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
  {
    res.success = true;
    return true;
  }

  bool set_control_types_callback(custom_msgs::SetControlTypes::Request& req, custom_msgs::SetControlTypes::Response& res)
  {
    res.success = true;
    return true;
  }

  bool set_pid_gains_callback(custom_msgs::SetPIDGains::Request& req, custom_msgs::SetPIDGains::Response& res)
  {
    res.success = true;
    return true;
  }

};

int main(int argc, char **argv)
{
  Controls controls = Controls(argc, argv);
  ros::spin();
  return 0;
}
