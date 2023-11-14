#ifndef CONTROLS_H
#define CONTROLS_H

#include <map>
#include <string>
#include <array>
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
#include "controls_utils.h"

class Controls
{
public:
    static const int THRUSTER_ALLOCS_RATE = 20;

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
    int num_thrusters;

    std::map<AxisEnum, ControlTypesEnum> control_types;
    std::map<AxisEnum, double> position_pid_outputs;
    std::map<AxisEnum, double> velocity_pid_outputs;
    std::map<AxisEnum, double> desired_power;

    Controls(int argc, char **argv);
    void desired_position_callback(const geometry_msgs::Pose msg);
    void desired_velocity_callback(const geometry_msgs::Twist msg);
    void desired_power_callback(const geometry_msgs::Twist msg);
    void state_callback(const nav_msgs::Odometry msg);
    bool enable_controls_callback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
    bool set_control_types_callback(custom_msgs::SetControlTypes::Request &req, custom_msgs::SetControlTypes::Response &res);
    bool set_pid_gains_callback(custom_msgs::SetPIDGains::Request &req, custom_msgs::SetPIDGains::Response &res);
    void run();
};

#endif