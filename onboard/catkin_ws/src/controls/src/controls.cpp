#include <unordered_map>
#include <string>
#include <array>
#include <memory>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
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
#include "pid_manager.h"
#include "controls_utils.h"
#include "controls.h"

Controls::Controls(int argc, char **argv, ros::NodeHandle &nh, std::unique_ptr<tf2_ros::Buffer> tfl_buffer)
{
    // Get parameters from launch file
    ros::param::get("~sim", sim);
    ros::param::get("~enable_position_pid", enable_position_pid);
    ros::param::get("~enable_velocity_pid", enable_velocity_pid);

    // Initialize TransformListener
    this->tfl_buffer = std::move(tfl_buffer);

    // Initialize desired position to have valid orientation
    desired_position.orientation.w = 1.0;

    // Subscribe to input topics
    desired_position_sub = nh.subscribe("controls/desired_position", 1, &Controls::desired_position_callback, this);
    desired_velocity_sub = nh.subscribe("controls/desired_velocity", 1, &Controls::desired_velocity_callback, this);
    desired_power_sub = nh.subscribe("controls/desired_power", 1, &Controls::desired_power_callback, this);
    state_sub = nh.subscribe("state", 1, &Controls::state_callback, this);

    // Advertise input services
    enable_controls_srv = nh.advertiseService("controls/enable", &Controls::enable_controls_callback, this);
    set_control_types_srv = nh.advertiseService("controls/set_control_types", &Controls::set_control_types_callback, this);
    set_pid_gains_srv = nh.advertiseService("controls/set_pid_gains", &Controls::set_pid_gains_callback, this);
    reset_pid_loops_srv = nh.advertiseService("controls/reset_pid_loops", &Controls::reset_pid_loops_callback, this);
    set_static_power_global_srv = nh.advertiseService("controls/set_static_power_global", &Controls::set_static_power_global_callback, this);
    set_power_scale_factor_srv = nh.advertiseService("controls/set_power_scale_factor", &Controls::set_power_scale_factor_callback, this);

    // Initialize publishers for output topics
    thruster_allocs_pub = nh.advertise<custom_msgs::ThrusterAllocs>("controls/thruster_allocs", 1);
    desired_thruster_allocs_pub = nh.advertise<custom_msgs::ThrusterAllocs>("controls/desired_thruster_allocs", 1);
    unconstrained_thruster_allocs_pub = nh.advertise<custom_msgs::ThrusterAllocs>("controls/unconstrained_thruster_allocs", 1);
    set_power_pub = nh.advertise<geometry_msgs::Twist>("controls/set_power", 1);
    actual_power_pub = nh.advertise<geometry_msgs::Twist>("controls/actual_power", 1);
    pid_gains_pub = nh.advertise<custom_msgs::PIDGains>("controls/pid_gains", 1);
    control_types_pub = nh.advertise<custom_msgs::ControlTypes>("controls/control_types", 1);
    position_error_pub = nh.advertise<geometry_msgs::Twist>("controls/position_error", 1);
    velocity_error_pub = nh.advertise<geometry_msgs::Twist>("controls/velocity_error", 1);
    status_pub = nh.advertise<std_msgs::Bool>("controls/status", 1);
    delta_time_pub = nh.advertise<std_msgs::Float64>("controls/delta_time", 1);
    static_power_global_pub = nh.advertise<geometry_msgs::Vector3>("controls/static_power_global", 1);
    static_power_local_pub = nh.advertise<geometry_msgs::Vector3>("controls/static_power_local", 1);
    power_scale_factor_pub = nh.advertise<std_msgs::Float64>("controls/power_scale_factor", 1);

    // Use desired pose as the default control type for all axes
    for (const AxesEnum &axis : AXES)
        control_types[axis] = ControlTypesEnum::DESIRED_POSE;

    // Get PID gains from robot config file
    std::string wrench_matrix_file_path;
    std::string wrench_matrix_pinv_file_path;
    ControlsUtils::read_robot_config(ROBOT_CONFIG_FILE_PATH, all_pid_gains, static_power_global, power_scale_factor,
                                     wrench_matrix_file_path, wrench_matrix_pinv_file_path);

    // Instantiate PID managers for each PID loop type
    for (const PIDLoopTypesEnum &pid_loop_type : PID_LOOP_TYPES)
        pid_managers[pid_loop_type] = PIDManager(all_pid_gains[pid_loop_type]);

    // Instantiate thruster allocator
    thruster_allocator = ThrusterAllocator(wrench_matrix_file_path, wrench_matrix_pinv_file_path);
}

void Controls::desired_position_callback(const geometry_msgs::Pose msg)
{
    if (ControlsUtils::quaternion_valid(msg.orientation))
        desired_position = msg;
    else
        ROS_WARN("Invalid desired position orientation. Quaternion must have length 1.");
}

void Controls::desired_velocity_callback(const geometry_msgs::Twist msg)
{
    desired_velocity = msg;
}

void Controls::desired_power_callback(const geometry_msgs::Twist msg)
{
    // Make sure desired power is within range [-1, 1]
    if (ControlsUtils::twist_in_range(msg, -1, 1))
        ControlsUtils::twist_to_map(msg, desired_power);
    else
        ROS_WARN("Invalid desired power. Desired power must be within range [-1, 1].");
}

void Controls::state_callback(const nav_msgs::Odometry msg)
{
    state = msg;

    // Get current time, compute delta time, and update last state message time
    // If last state message time is zero, then this is the first state message received, so delta time is zero
    ros::Time current_time = ros::Time::now();
    double delta_time = last_state_msg_time.is_zero() ? 0 : (current_time - last_state_msg_time).toSec();
    last_state_msg_time = current_time;

    // Publish delta time
    std_msgs::Float64 delta_time_msg;
    delta_time_msg.data = delta_time;
    delta_time_pub.publish(delta_time_msg);

    if (delta_time <= 0.0)
        return;

    // Get transform from odom to base_link
    geometry_msgs::TransformStamped transformStamped;
    try
    {
        transformStamped = tfl_buffer->lookupTransform("odom", "base_link", ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("Could not get transform from odom to base_link. %s", ex.what());
        return;
    }

    // Compute position error
    geometry_msgs::PoseStamped desired_position_stamped;
    desired_position_stamped.pose = desired_position;
    desired_position_stamped.header.frame_id = "odom";
    geometry_msgs::PoseStamped position_error;
    tf2::doTransform(desired_position_stamped, position_error, transformStamped);

    // Compute velocity error
    // No transform is required because the state and desired velocities are in the base_link frame
    geometry_msgs::Twist velocity_error;
    velocity_error.linear.x = desired_velocity.linear.x - state.twist.twist.linear.x;
    velocity_error.linear.y = desired_velocity.linear.y - state.twist.twist.linear.y;
    velocity_error.linear.z = desired_velocity.linear.z - state.twist.twist.linear.z;
    velocity_error.angular.x = desired_velocity.angular.x - state.twist.twist.angular.x;
    velocity_error.angular.y = desired_velocity.angular.y - state.twist.twist.angular.y;
    velocity_error.angular.z = desired_velocity.angular.z - state.twist.twist.angular.z;

    // Convert error messages to maps
    geometry_msgs::Twist position_error_msg;
    ControlsUtils::pose_to_twist(position_error.pose, position_error_msg);

    std::unordered_map<AxesEnum, double> position_error_map;
    ControlsUtils::twist_to_map(position_error_msg, position_error_map);

    std::unordered_map<AxesEnum, double> velocity_error_map;
    ControlsUtils::twist_to_map(velocity_error, velocity_error_map);

    // Get delta time map
    std::unordered_map<AxesEnum, double> delta_time_map;
    ControlsUtils::populate_axes_map(delta_time_map, delta_time);

    // Rotate static power to equivalent vector in robot's local frame
    tf2::Quaternion orientation_tf2;
    tf2::fromMsg(state.pose.pose.orientation, orientation_tf2);
    tf2::Vector3 static_power_local = quatRotate(orientation_tf2.inverse(), static_power_global);

    std::unordered_map<AxesEnum, double> static_power_local_map;
    ControlsUtils::tf_linear_vector_to_map(static_power_local, static_power_local_map);

    // Publish static power rotated
    geometry_msgs::Vector3 static_power_local_msg = tf2::toMsg(static_power_local);
    static_power_local_pub.publish(static_power_local_msg);

    // Run PID loops
    if (enable_position_pid)
        pid_managers[PIDLoopTypesEnum::POSITION].run_loops(position_error_map, delta_time_map,
                                                           static_power_local_map, position_pid_outputs);

    if (enable_velocity_pid)
        pid_managers[PIDLoopTypesEnum::VELOCITY].run_loops(velocity_error_map, delta_time_map,
                                                           static_power_local_map, velocity_pid_outputs);

    // Publish error messages
    position_error_pub.publish(position_error_msg);
    velocity_error_pub.publish(velocity_error);
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
    res.success = ControlsUtils::update_pid_loops_axes_gains_map(all_pid_gains, req.pid_gains);

    // Update robot config file if PID gains were updated successfully
    // Logs an error if file could not be updated and shuts down the node.
    // Why throw an exception if file could not be updated, but not if PID gains were invalid?
    // Answer: because if the PID gains were invalid, then we give the user another chance to enter valid PID gains
    // and don't make any changes. If the file could not be updated successfully, then we don't want to give the user
    // another chance to enter valid PID gains because the file will likely still not be updated successfully and
    // despite being incomplete, we have no way of undoing the changes (if any) that were made to the file. It is likely
    // users will make frequent mistakes when calling this service, but it is unlikely writing to config file will
    // frequently fail.
    if (res.success)
        ControlsUtils::update_robot_pid_gains(ROBOT_CONFIG_FILE_PATH, all_pid_gains);

    res.message = res.success ? "Updated PID gains successfully." : "Failed to update PID gains. One or more PID gains was invalid.";

    return true;
}

bool Controls::reset_pid_loops_callback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    // Reset all active PID loops
    for (const AxesEnum &axis : AXES)
    {
        if (control_types[axis] == ControlTypesEnum::DESIRED_POSE)
            pid_managers[PIDLoopTypesEnum::POSITION].reset(axis);
        else if (control_types[axis] == ControlTypesEnum::DESIRED_TWIST)
            pid_managers[PIDLoopTypesEnum::VELOCITY].reset(axis);
    }

    res.success = true;
    res.message = "Reset PID loops successfully.";
    return true;
}

bool Controls::set_static_power_global_callback(custom_msgs::SetStaticPower::Request &req, custom_msgs::SetStaticPower::Response &res)
{
    tf2::fromMsg(req.static_power, static_power_global);

    // Update robot config file if static power was updated successfully
    // Logs an error if file could not be updated and shuts down the node.
    // Why throw an exception if file could not be updated, but not if static power was invalid?
    // Answer: because if the static power was invalid, then we give the user another chance to enter valid static power
    // and don't make any changes. If the file could not be updated successfully, then we don't want to give the user
    // another chance to enter valid static power because the file will likely still not be updated successfully and
    // despite being incomplete, we have no way of undoing the changes (if any) that were made to the file. It is likely
    // users will make frequent mistakes when calling this service, but it is unlikely writing to config file will
    // frequently fail.
    ControlsUtils::update_robot_static_power_global(ROBOT_CONFIG_FILE_PATH, static_power_global);

    res.success = true;
    res.message = res.success ? "Updated static power successfully." : "Failed to update static power. Static power was invalid.";

    return true;
}

bool Controls::set_power_scale_factor_callback(custom_msgs::SetPowerScaleFactor::Request &req, custom_msgs::SetPowerScaleFactor::Response &res)
{
    power_scale_factor = req.power_scale_factor;

    ControlsUtils::update_robot_power_scale_factor(ROBOT_CONFIG_FILE_PATH, power_scale_factor);

    res.success = true;
    res.message = res.success ? "Updated power scale factor successfully." : "Failed to update power scale factor. Power scale factor was invalid.";

    return true;
}

void Controls::run()
{
    ros::Rate rate(THRUSTER_ALLOCS_RATE);

    Eigen::VectorXd set_power(AXES_COUNT);
    Eigen::VectorXd actual_power;
    Eigen::VectorXd unconstrained_allocs;
    Eigen::VectorXd constrained_allocs;

    custom_msgs::ThrusterAllocs unconstrained_t;
    custom_msgs::ThrusterAllocs constrained_t;
    geometry_msgs::Twist set_power_msg;
    geometry_msgs::Twist actual_power_msg;
    custom_msgs::ControlTypes control_types_msg;
    std_msgs::Bool status_msg;
    custom_msgs::PIDGains pid_gains_msg;
    geometry_msgs::Vector3 static_power_global_msg;
    std_msgs::Float64 power_scale_factor_msg;

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

        thruster_allocator.allocate_thrusters(set_power, power_scale_factor, unconstrained_allocs,
                                              constrained_allocs, actual_power);

        ControlsUtils::eigen_vector_to_thruster_allocs(constrained_allocs, constrained_t);

        if (controls_enabled)
            thruster_allocs_pub.publish(constrained_t);

        desired_thruster_allocs_pub.publish(constrained_t);

        ControlsUtils::eigen_vector_to_thruster_allocs(unconstrained_allocs, unconstrained_t);
        unconstrained_thruster_allocs_pub.publish(unconstrained_t);

        ControlsUtils::eigen_vector_to_twist(set_power, set_power_msg);
        set_power_pub.publish(set_power_msg);

        ControlsUtils::eigen_vector_to_twist(actual_power, actual_power_msg);
        actual_power_pub.publish(actual_power_msg);

        ControlsUtils::map_to_control_types(control_types, control_types_msg);
        control_types_pub.publish(control_types_msg);

        status_msg.data = controls_enabled;
        status_pub.publish(status_msg);

        ControlsUtils::pid_loops_axes_gains_map_to_msg(all_pid_gains, pid_gains_msg);
        pid_gains_pub.publish(pid_gains_msg);

        static_power_global_msg = tf2::toMsg(static_power_global);
        static_power_global_pub.publish(static_power_global_msg);

        power_scale_factor_msg.data = power_scale_factor;
        power_scale_factor_pub.publish(power_scale_factor_msg);

        ros::spinOnce();

        rate.sleep();
    }
}

int main(int argc, char **argv)
{
    // Initialize ROS node
    ros::init(argc, argv, "controls");

    ros::NodeHandle nh;

    std::unique_ptr<tf2_ros::Buffer> tfl_buffer(new tf2_ros::Buffer());
    std::unique_ptr<tf2_ros::TransformListener> tfl(new tf2_ros::TransformListener(*tfl_buffer));

    Controls controls = Controls(argc, argv, nh, std::move(tfl_buffer));
    controls.run();

    return 0;
}
