#ifndef CONTROLS_H
#define CONTROLS_H

#include <unordered_map>
#include <string>
#include <array>
#include <memory>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <custom_msgs/ThrusterAllocs.h>
#include <custom_msgs/PIDGain.h>
#include <custom_msgs/PIDGains.h>
#include <custom_msgs/ControlTypes.h>
#include <custom_msgs/SetPIDGains.h>
#include <custom_msgs/SetControlTypes.h>
#include <custom_msgs/SetPowerScaleFactor.h>
#include <custom_msgs/SetStaticPower.h>
#include "thruster_allocator.h"
#include "controls_utils.h"
#include "pid_manager.h"

class Controls
{
private:
    static const int THRUSTER_ALLOCS_RATE = 20;

    bool sim = false;
    bool enable_position_pid = false;
    bool enable_velocity_pid = false;

    std::unique_ptr<tf2_ros::Buffer> tfl_buffer;

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
    ros::ServiceServer reset_pid_loops_srv;
    ros::ServiceServer set_static_power_global_srv;
    ros::ServiceServer set_power_scale_factor_srv;

    ros::Publisher thruster_allocs_pub;
    ros::Publisher desired_thruster_allocs_pub;
    ros::Publisher unconstrained_thruster_allocs_pub;
    ros::Publisher set_power_pub;
    ros::Publisher set_power_scaled_pub;
    ros::Publisher actual_power_pub;
    ros::Publisher set_scaled_actual_power_diff_pub;
    ros::Publisher pid_gains_pub;
    ros::Publisher control_types_pub;
    ros::Publisher position_efforts_pub;
    ros::Publisher velocity_efforts_pub;
    ros::Publisher position_error_pub;
    ros::Publisher velocity_error_pub;
    ros::Publisher position_pid_infos_pub;
    ros::Publisher velocity_pid_infos_pub;
    ros::Publisher status_pub;
    ros::Publisher delta_time_pub;
    ros::Publisher static_power_global_pub;
    ros::Publisher static_power_local_pub;
    ros::Publisher power_scale_factor_pub;

    bool controls_enabled = false;

    ros::Time last_state_msg_time;

    LoopsMap<PIDManager> pid_managers;

    AxesMap<double> actual_power_map;

    tf2::Vector3 static_power_global;
    tf2::Vector3 static_power_local;

    double power_scale_factor;

    ThrusterAllocator thruster_allocator;

    AxesMap<ControlTypesEnum> control_types;
    AxesMap<double> position_pid_outputs;
    AxesMap<double> velocity_pid_outputs;
    AxesMap<PIDInfo> position_pid_infos;
    AxesMap<PIDInfo> velocity_pid_infos;
    AxesMap<double> desired_power;

    void desired_position_callback(const geometry_msgs::Pose msg);
    void desired_velocity_callback(const geometry_msgs::Twist msg);
    void desired_power_callback(const geometry_msgs::Twist msg);
    void state_callback(const nav_msgs::Odometry msg);
    bool enable_controls_callback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
    bool set_control_types_callback(custom_msgs::SetControlTypes::Request &req, custom_msgs::SetControlTypes::Response &res);
    bool set_pid_gains_callback(custom_msgs::SetPIDGains::Request &req, custom_msgs::SetPIDGains::Response &res);
    bool reset_pid_loops_callback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool set_static_power_global_callback(custom_msgs::SetStaticPower::Request &req, custom_msgs::SetStaticPower::Response &res);
    bool set_power_scale_factor_callback(custom_msgs::SetPowerScaleFactor::Request &req, custom_msgs::SetPowerScaleFactor::Response &res);

public:
    Controls(int argc, char **argv, ros::NodeHandle &nh, std::unique_ptr<tf2_ros::Buffer> tfl_buffer);
    void run();
};

#endif