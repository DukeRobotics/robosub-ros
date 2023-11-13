#include <map>
#include <string>
#include <Eigen/Dense>
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
#include "thruster_allocator.h"

class Controls
{
private:
  bool update_control_types(custom_msgs::ControlTypes new_control_types)
  {
    // TODO: Make sure all values in new_control_types are from ControlTypes enum; if not, return false

    control_types["x"] = new_control_types.x;
    control_types["y"] = new_control_types.y;
    control_types["z"] = new_control_types.z;
    control_types["roll"] = new_control_types.roll;
    control_types["pitch"] = new_control_types.pitch;
    control_types["yaw"] = new_control_types.yaw;

    return true;
  }

public:
  static const int THRUSTER_ALLOCS_RATE;
  static const std::string AXES[6];

  bool debug;
  bool enable_position_pid;
  bool enable_velocity_pid;

  ThrusterAllocator thruster_allocator;

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

  ros::Publisher thruster_allocs_pub;
  ros::Publisher desired_thruster_allocs_pub;
  ros::Publisher set_power_pub;
  ros::Publisher pid_gains_pub;
  ros::Publisher control_types_pub;
  ros::Publisher position_error_pub;
  ros::Publisher velocity_error_pub;
  ros::Publisher status_pub;

  bool controls_enabled = false;
  std::map<std::string, int> control_types;
  int num_thrusters;

  Controls(int argc, char **argv):
    // Initialize thruster allocator
    // TODO: Get csv file paths from robot config file
    thruster_allocator("../config/oogway_wrench.csv", "../config/oogway_wrench_pinv.csv")
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

    // Initialize publishers for output topics
    thruster_allocs_pub = nh.advertise<custom_msgs::ThrusterAllocs>("controls/thruster_allocs", 1);
    desired_thruster_allocs_pub = nh.advertise<custom_msgs::ThrusterAllocs>("controls/desired_thruster_allocs", 1);
    set_power_pub = nh.advertise<geometry_msgs::Twist>("controls/set_power", 1);
    pid_gains_pub = nh.advertise<custom_msgs::PIDGains>("controls/pid_gains", 1);
    control_types_pub = nh.advertise<custom_msgs::ControlTypes>("controls/control_types", 1);
    position_error_pub = nh.advertise<geometry_msgs::Pose>("controls/position/error", 1);
    velocity_error_pub = nh.advertise<geometry_msgs::Twist>("controls/velocity/error", 1);
    status_pub = nh.advertise<std_msgs::Bool>("controls/status", 1);

    // Use desired pose as the default control type for all axes
    for (const auto &axis : AXES)
    {
      control_types[axis] = custom_msgs::ControlTypes::DESIRED_POSE;
    }

    // TODO: Get number of thrusters from robot config file
    num_thrusters = 8;
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

  bool enable_controls_callback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
  {
    controls_enabled = req.data;
    res.success = true;
    return true;
  }

  bool set_control_types_callback(custom_msgs::SetControlTypes::Request &req, custom_msgs::SetControlTypes::Response &res)
  {
    res.success = update_control_types(req.control_types);
    res.message = res.success ? "Updated control types successfully." : "One or more control types was invalid.";
    return true;
  }

  bool set_pid_gains_callback(custom_msgs::SetPIDGains::Request &req, custom_msgs::SetPIDGains::Response &res)
  {
    res.success = true;
    return true;
  }

  void run()
  {
    ros::Rate rate(THRUSTER_ALLOCS_RATE);

    while(ros::ok()) {
      // TODO: compute set_power using control_types and PID outputs/desired power
      Eigen::VectorXd set_power(6);
      set_power << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0;

      Eigen::VectorXd actual_power;
      Eigen::VectorXd allocs;

      thruster_allocator.allocate_thrusters(set_power, &allocs, &actual_power);

      custom_msgs::ThrusterAllocs t;
      for (int i = 0; i < num_thrusters; i++) {
        t.allocs[i] = allocs(i);
      }

      if (controls_enabled) {
        thruster_allocs_pub.publish(t);
      }

      desired_thruster_allocs_pub.publish(t);

      ros::spinOnce();

      rate.sleep();
    }
  }
};

const int Controls::THRUSTER_ALLOCS_RATE = 20;
const std::string Controls::AXES[6] = {"x", "y", "z", "roll", "pitch", "yaw"};

int main(int argc, char **argv)
{
  Controls controls = Controls(argc, argv);
  controls.run();
  return 0;
}
