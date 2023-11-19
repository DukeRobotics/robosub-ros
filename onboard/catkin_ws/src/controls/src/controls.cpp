#include <unordered_map>
#include <string>
#include <array>
#include <memory>
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
#include "pid_manager.h"
#include "controls_utils.h"
#include "controls.h"

Controls::Controls(int argc, char **argv, ros::NodeHandle &nh, std::unique_ptr<tf::TransformListener> tf_listener)
{

    // Get parameters from launch file
    nh.param<bool>("sim", sim, false);
    nh.param<bool>("debug", debug, false);
    nh.param<bool>("enable_position_pid", enable_position_pid, false);
    nh.param<bool>("enable_velocity_pid", enable_velocity_pid, false);

    this->tf_listener = std::move(tf_listener);

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
    actual_power_pub = nh.advertise<geometry_msgs::Twist>("controls/actual_power", 1);
    pid_gains_pub = nh.advertise<custom_msgs::PIDGains>("controls/pid_gains", 1);
    control_types_pub = nh.advertise<custom_msgs::ControlTypes>("controls/control_types", 1);
    position_error_pub = nh.advertise<geometry_msgs::Pose>("controls/position/error", 1);
    velocity_error_pub = nh.advertise<geometry_msgs::Twist>("controls/velocity/error", 1);
    status_pub = nh.advertise<std_msgs::Bool>("controls/status", 1);

    // Use desired pose as the default control type for all axes
    for (const AxisEnum &axis : AXES)
        control_types[axis] = ControlTypesEnum::DESIRED_POSE;

    // TODO: Get PID gains from robot config file

    // TODO: Instantiate PID managers for each PID loop type
    for(const PIDLoopTypesEnum& pid_loop_type : PID_LOOP_TYPES)
        pid_managers[pid_loop_type] = PIDManager(all_pid_gains[pid_loop_type]);

    // TODO: Get csv file paths from robot config file
    thruster_allocator = ThrusterAllocator(
        "/root/dev/robosub-ros/onboard/catkin_ws/src/controls/config/oogway_wrench.csv",
        "/root/dev/robosub-ros/onboard/catkin_ws/src/controls/config/oogway_wrench_pinv.csv");
}

void Controls::desired_position_callback(const geometry_msgs::Pose msg)
{
    desired_position = msg;
}

void Controls::desired_velocity_callback(const geometry_msgs::Twist msg)
{
    desired_velocity = msg;
}

void Controls::desired_power_callback(const geometry_msgs::Twist msg)
{
    ControlsUtils::twist_to_map(msg, desired_power);
}

void Controls::state_callback(const nav_msgs::Odometry msg)
{
    state = msg;
    // TODO: compute position and velocity errors and run PID controllers
    // TODO: publish position and velocity errors
}

bool Controls::enable_controls_callback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    controls_enabled = req.data;
    res.success = true;
    return true;
}

bool Controls::set_control_types_callback(custom_msgs::SetControlTypes::Request &req, custom_msgs::SetControlTypes::Response &res)
{
    res.success = ControlsUtils::control_types_to_map(req.control_types, control_types);
    res.message = res.success ? "Updated control types successfully." : "Failed to update control types. One or more control types was invalid.";
    return true;
}

bool Controls::set_pid_gains_callback(custom_msgs::SetPIDGains::Request &req, custom_msgs::SetPIDGains::Response &res)
{
    // TODO: Update PID gains
    res.success = true;
    return true;
}

void Controls::run()
{
    ros::Rate rate(THRUSTER_ALLOCS_RATE);

    Eigen::VectorXd set_power(AXES_COUNT);
    Eigen::VectorXd actual_power;
    Eigen::VectorXd allocs;

    custom_msgs::ThrusterAllocs t;
    geometry_msgs::Twist set_power_msg;
    geometry_msgs::Twist actual_power_msg;
    custom_msgs::ControlTypes control_types_msg;
    std_msgs::Bool status_msg;

    while (ros::ok())
    {
        for (int i = 0; i < AXES_COUNT; i++)
        {
            switch (control_types[AXES[i]])
            {
            case custom_msgs::ControlTypes::DESIRED_POSE:
                set_power[i] = position_pid_outputs[AXES[i]];
                break;
            case custom_msgs::ControlTypes::DESIRED_TWIST:
                set_power[i] = velocity_pid_outputs[AXES[i]];
                break;
            case custom_msgs::ControlTypes::DESIRED_POWER:
                set_power[i] = desired_power[AXES[i]];
                break;
            }
        }

        thruster_allocator.allocate_thrusters(set_power, allocs, actual_power);

        t.allocs.clear();
        for (int i = 0; i < allocs.rows(); i++)
            t.allocs.push_back(allocs[i]);

        if (controls_enabled)
            thruster_allocs_pub.publish(t);

        desired_thruster_allocs_pub.publish(t);

        ControlsUtils::eigen_vector_to_twist(set_power, set_power_msg);
        set_power_pub.publish(set_power_msg);

        ControlsUtils::eigen_vector_to_twist(actual_power, actual_power_msg);
        actual_power_pub.publish(actual_power_msg);

        ControlsUtils::map_to_control_types(control_types, control_types_msg);
        control_types_pub.publish(control_types_msg);

        status_msg.data = controls_enabled;
        status_pub.publish(status_msg);

        // TODO: Publish PID gains

        ros::spinOnce();

        rate.sleep();
    }
}

int main(int argc, char **argv)
{
    // Initialize ROS node
    ros::init(argc, argv, "controls");

    ros::NodeHandle nh;

    std::unique_ptr<tf::TransformListener> tf_listener(new tf::TransformListener());

    Controls controls = Controls(argc, argv, nh, std::move(tf_listener));
    controls.run();

    return 0;
}
