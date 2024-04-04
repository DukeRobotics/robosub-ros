#include "controls.h"

#include <custom_msgs/ControlTypes.h>
#include <custom_msgs/PIDAxesInfo.h>
#include <custom_msgs/PIDGain.h>
#include <custom_msgs/PIDGains.h>
#include <custom_msgs/SetControlTypes.h>
#include <custom_msgs/SetPIDGains.h>
#include <custom_msgs/SetPowerScaleFactor.h>
#include <custom_msgs/SetStaticPower.h>
#include <custom_msgs/ThrusterAllocs.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Dense>
#include <array>
#include <memory>
#include <string>
#include <unordered_map>

#include "controls_types.h"
#include "controls_utils.h"
#include "pid_manager.h"
#include "thruster_allocator.h"

const int Controls::THRUSTER_ALLOCS_RATE = 20;

Controls::Controls(int argc, char **argv, ros::NodeHandle &nh, std::unique_ptr<tf2_ros::Buffer> tf_buffer) {
    // Get parameters from launch file
    ros::param::param("~sim", sim, false);
    ros::param::param("~enable_position_pid", enable_position_pid, false);
    ros::param::param("~enable_velocity_pid", enable_velocity_pid, false);
    ros::param::param("~cascaded_pid", cascaded_pid, false);

    // Initialize controls to be disabled
    controls_enabled = false;

    // Store transform buffer
    this->tf_buffer = std::move(tf_buffer);

    // Initialize desired position to have valid orientation
    desired_position.orientation.w = 1.0;

    // Use desired position as the default control type for all axes
    ControlsUtils::populate_axes_map<ControlTypesEnum>(ControlTypesEnum::DESIRED_POSITION, control_types);

    // Get paths to wrench matrix and its pseudoinverse from robot config file
    std::string wrench_matrix_file_path;
    std::string wrench_matrix_pinv_file_path;

    // Get PID configuration parameters from robot config file
    LoopsMap<AxesMap<double>> loops_axes_control_effort_mins;
    LoopsMap<AxesMap<double>> loops_axes_control_effort_maxes;
    LoopsMap<AxesMap<PIDDerivativeTypesEnum>> loops_axes_derivative_types;
    LoopsMap<AxesMap<double>> loops_axes_error_ramp_rates;
    LoopsMap<AxesMap<PIDGainsMap>> loops_axes_pid_gains;

    // Read robot config file and populate config variables
    ControlsUtils::read_robot_config(cascaded_pid, loops_axes_control_effort_mins, loops_axes_control_effort_maxes,
                                     loops_axes_derivative_types, loops_axes_error_ramp_rates, loops_axes_pid_gains,
                                     desired_power_min, desired_power_max, static_power_global, power_scale_factor,
                                     wrench_matrix_file_path, wrench_matrix_pinv_file_path);

    // Ensure that desired power min is less than or equal to desired power max for each axis
    for (const AxesEnum &axis : AXES)
        ROS_ASSERT_MSG(
            desired_power_min.at(axis) <= desired_power_max.at(axis),
            "Invalid desired power min and max for axis %s. Desired power min must be less than or equal to max.",
            AXES_NAMES.at(axis).c_str());

    // Instantiate PID managers for each PID loop type
    for (const PIDLoopTypesEnum &loop : PID_LOOP_TYPES)
        pid_managers[loop] = PIDManager(loops_axes_control_effort_mins.at(loop),
                                        loops_axes_control_effort_maxes.at(loop), loops_axes_derivative_types.at(loop),
                                        loops_axes_error_ramp_rates.at(loop), loops_axes_pid_gains.at(loop));
    // Initialize static power local to zero
    static_power_local = tf2::Vector3(0, 0, 0);

    // Initialize axes maps to zero
    ControlsUtils::populate_axes_map<double>(0, position_pid_outputs);
    ControlsUtils::populate_axes_map<double>(0, velocity_pid_outputs);
    ControlsUtils::populate_axes_map<double>(0, desired_power);

    // Instantiate thruster allocator
    thruster_allocator = ThrusterAllocator(wrench_matrix_file_path, wrench_matrix_pinv_file_path);

    // Subscribe to input topics
    state_sub = nh.subscribe("state", 1, &Controls::state_callback, this);
    desired_position_sub = nh.subscribe("controls/desired_position", 1, &Controls::desired_position_callback, this);
    desired_velocity_sub = nh.subscribe("controls/desired_velocity", 1, &Controls::desired_velocity_callback, this);
    desired_power_sub = nh.subscribe("controls/desired_power", 1, &Controls::desired_power_callback, this);

    // Advertise input services
    enable_controls_srv = nh.advertiseService("controls/enable", &Controls::enable_controls_callback, this);
    set_control_types_srv =
        nh.advertiseService("controls/set_control_types", &Controls::set_control_types_callback, this);
    set_pid_gains_srv = nh.advertiseService("controls/set_pid_gains", &Controls::set_pid_gains_callback, this);
    reset_pid_loops_srv = nh.advertiseService("controls/reset_pid_loops", &Controls::reset_pid_loops_callback, this);
    set_static_power_global_srv =
        nh.advertiseService("controls/set_static_power_global", &Controls::set_static_power_global_callback, this);
    set_power_scale_factor_srv =
        nh.advertiseService("controls/set_power_scale_factor", &Controls::set_power_scale_factor_callback, this);

    // Initialize publishers for output topics
    thruster_allocs_pub = nh.advertise<custom_msgs::ThrusterAllocs>("controls/thruster_allocs", 1);
    constrained_thruster_allocs_pub =
        nh.advertise<custom_msgs::ThrusterAllocs>("controls/constrained_thruster_allocs", 1);
    unconstrained_thruster_allocs_pub =
        nh.advertise<custom_msgs::ThrusterAllocs>("controls/unconstrained_thruster_allocs", 1);
    base_power_pub = nh.advertise<geometry_msgs::Twist>("controls/base_power", 1);
    set_power_unscaled_pub = nh.advertise<geometry_msgs::Twist>("controls/set_power_unscaled", 1);
    set_power_pub = nh.advertise<geometry_msgs::Twist>("controls/set_power", 1);
    actual_power_pub = nh.advertise<geometry_msgs::Twist>("controls/actual_power", 1);
    power_disparity_pub = nh.advertise<geometry_msgs::Twist>("controls/power_disparity", 1);
    power_disparity_norm_pub = nh.advertise<std_msgs::Float64>("controls/power_disparity_norm", 1);
    pid_gains_pub = nh.advertise<custom_msgs::PIDGains>("controls/pid_gains", 1);
    control_types_pub = nh.advertise<custom_msgs::ControlTypes>("controls/control_types", 1);
    position_efforts_pub = nh.advertise<geometry_msgs::Twist>("controls/position_efforts", 1);
    velocity_efforts_pub = nh.advertise<geometry_msgs::Twist>("controls/velocity_efforts", 1);
    position_error_pub = nh.advertise<geometry_msgs::Twist>("controls/position_error", 1);
    velocity_error_pub = nh.advertise<geometry_msgs::Twist>("controls/velocity_error", 1);
    position_pid_infos_pub = nh.advertise<custom_msgs::PIDAxesInfo>("controls/position_pid_infos", 1);
    velocity_pid_infos_pub = nh.advertise<custom_msgs::PIDAxesInfo>("controls/velocity_pid_infos", 1);
    status_pub = nh.advertise<std_msgs::Bool>("controls/status", 1);
    delta_time_pub = nh.advertise<std_msgs::Float64>("controls/delta_time", 1);
    static_power_global_pub = nh.advertise<geometry_msgs::Vector3>("controls/static_power_global", 1);
    static_power_local_pub = nh.advertise<geometry_msgs::Vector3>("controls/static_power_local", 1);
    power_scale_factor_pub = nh.advertise<std_msgs::Float64>("controls/power_scale_factor", 1);
}

void Controls::desired_position_callback(const geometry_msgs::Pose msg) {
    // Make sure desired position orientation quaternion has length 1
    if (ControlsUtils::quaternion_valid(msg.orientation))
        desired_position = msg;
    else
        ROS_WARN("Invalid desired position orientation. Quaternion must have length 1.");
}

void Controls::desired_velocity_callback(const geometry_msgs::Twist msg) { desired_velocity = msg; }

void Controls::desired_power_callback(const geometry_msgs::Twist msg) {
    AxesMap<double> new_desired_power;
    ControlsUtils::twist_to_map(msg, new_desired_power);

    // Make sure desired power is within the limits specified in the config file for each axis
    // If desired power is invalid, then warn user and don't update desired power
    for (const AxesEnum &axis : AXES) {
        if (new_desired_power.at(axis) < desired_power_min.at(axis) ||
            new_desired_power.at(axis) > desired_power_max.at(axis)) {
            ROS_WARN(
                "Invalid desired power of %f for axis %s. Desired power for axis %s must be within range [%f, %f].",
                new_desired_power.at(axis), AXES_NAMES.at(axis).c_str(), AXES_NAMES.at(axis).c_str(),
                desired_power_min.at(axis), desired_power_max.at(axis));
            return;
        }
    }

    // New desired power is within limits, so update desired power
    desired_power = new_desired_power;
}

void Controls::state_callback(const nav_msgs::Odometry msg) {
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

    // Don't run PID loops if delta_time is nonpositive
    // Only occurs on first state message received
    if (delta_time <= 0.0) return;

    // Get transform from odom to base_link
    geometry_msgs::TransformStamped transformStamped;
    try {
        transformStamped = tf_buffer->lookupTransform("base_link", "odom", ros::Time(0));
    } catch (tf2::TransformException &ex) {
        ROS_WARN("Could not get transform from odom to base_link. %s", ex.what());
        return;
    }

    // Compute position error
    geometry_msgs::PoseStamped desired_position_stamped;
    desired_position_stamped.pose = desired_position;
    desired_position_stamped.header.frame_id = "odom";
    geometry_msgs::PoseStamped position_error;
    tf2::doTransform(desired_position_stamped, position_error, transformStamped);

    // Convert position error pose to twist
    geometry_msgs::Twist position_error_msg;
    ControlsUtils::pose_to_twist(position_error.pose, position_error_msg);

    // Publish position error message
    position_error_pub.publish(position_error_msg);

    // Convert position error twist to map
    AxesMap<double> position_error_map;
    ControlsUtils::twist_to_map(position_error_msg, position_error_map);

    // Get delta time map
    AxesMap<double> delta_time_map;
    ControlsUtils::populate_axes_map<double>(delta_time, delta_time_map);

    // Get velocity map
    AxesMap<double> velocity_map;
    ControlsUtils::twist_to_map(state.twist.twist, velocity_map);

    // Copy velocity map to position PID provided derivatives map and negate all entries
    // Entries are negated because the derivative of position error is negative velocity
    AxesMap<double> position_pid_provided_derivatives(velocity_map);
    ControlsUtils::scale_axes_map(-1, position_pid_provided_derivatives);

    // Run position PID loop
    if (enable_position_pid)
        pid_managers.at(PIDLoopTypesEnum::POSITION)
            .run_loop(position_error_map, delta_time_map, position_pid_outputs, position_pid_infos,
                      position_pid_provided_derivatives);

    // Publish position control efforts
    geometry_msgs::Twist position_efforts_msg;
    ControlsUtils::map_to_twist(position_pid_outputs, position_efforts_msg);
    position_efforts_pub.publish(position_efforts_msg);

    // Publish position PID infos
    custom_msgs::PIDAxesInfo position_pid_infos_msg;
    ControlsUtils::pid_axes_map_info_struct_to_msg(position_pid_infos, position_pid_infos_msg);
    position_pid_infos_pub.publish(position_pid_infos_msg);

    // Get desired velocity map
    AxesMap<double> desired_velocity_map;
    ControlsUtils::twist_to_map(desired_velocity, desired_velocity_map);

    // Get velocity error map
    // For each axis, if PID is cascaded and control type is position, then use position PID output as velocity setpoint
    // Otherwise, use desired velocity as velocity setpoint
    AxesMap<double> velocity_error_map;
    for (const AxesEnum &axis : AXES) {
        double velocity_setpoint = (cascaded_pid && control_types.at(axis) == ControlTypesEnum::DESIRED_POSITION)
                                       ? position_pid_outputs.at(axis)
                                       : desired_velocity_map.at(axis);
        velocity_error_map[axis] = velocity_setpoint - velocity_map[axis];
    }

    // Publish velocity error
    geometry_msgs::Twist velocity_error;
    ControlsUtils::map_to_twist(velocity_error_map, velocity_error);
    velocity_error_pub.publish(velocity_error);

    // Copy actual power map to velocity PID provided derivatives map and negate all entries
    // Entries are negated because the derivative of velocity error is negative acceleration
    AxesMap<double> velocity_pid_provided_derivatives(actual_power_map);
    ControlsUtils::scale_axes_map(-1, velocity_pid_provided_derivatives);

    // Run velocity PID loop
    if (enable_velocity_pid)
        pid_managers.at(PIDLoopTypesEnum::VELOCITY)
            .run_loop(velocity_error_map, delta_time_map, velocity_pid_outputs, velocity_pid_infos,
                      velocity_pid_provided_derivatives);

    // Publish velocity control efforts
    geometry_msgs::Twist velocity_efforts_msg;
    ControlsUtils::map_to_twist(velocity_pid_outputs, velocity_efforts_msg);
    velocity_efforts_pub.publish(velocity_efforts_msg);

    // Publish velocity PID infos
    custom_msgs::PIDAxesInfo velocity_pid_infos_msg;
    ControlsUtils::pid_axes_map_info_struct_to_msg(velocity_pid_infos, velocity_pid_infos_msg);
    velocity_pid_infos_pub.publish(velocity_pid_infos_msg);

    // Rotate static power global to equivalent vector in robot's local frame
    // Given vector v in base_link frame, v * state.pose.pose.orientation = v', where v' is the vector in the odom frame
    // that points in the same direction as v in the base_link frame, as seen by an external observer.
    // Here, we have static_power_global, a vector in the odom frame, and we would like static_power_local, a vector
    // in the base_link frame, that points in the same direction as static_power_global in the odom frame. Thus, we
    // are performing the inverse of the operation described above.
    // static_power_global * state.pose.pose.orientation.inverse() = static_power_local
    tf2::Quaternion orientation_tf2;
    tf2::fromMsg(state.pose.pose.orientation, orientation_tf2);
    static_power_local = quatRotate(orientation_tf2.inverse(), static_power_global);

    // Publish static power local
    geometry_msgs::Vector3 static_power_local_msg = tf2::toMsg(static_power_local);
    static_power_local_pub.publish(static_power_local_msg);
}

bool Controls::enable_controls_callback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
    controls_enabled = req.data;
    res.success = true;
    res.message = controls_enabled ? "Controls enabled." : "Controls disabled.";
    return true;
}

bool Controls::set_control_types_callback(custom_msgs::SetControlTypes::Request &req,
                                          custom_msgs::SetControlTypes::Response &res) {
    res.success = ControlsUtils::control_types_to_map(req.control_types, control_types);
    res.message = res.success ? "Updated control types successfully."
                              : "Failed to update control types. One or more control types was invalid.";
    return true;
}

bool Controls::set_pid_gains_callback(custom_msgs::SetPIDGains::Request &req, custom_msgs::SetPIDGains::Response &res) {
    res.success = ControlsUtils::pid_gains_valid(req.pid_gains);

    // If PID gains were valid, then update PID gains in PID managers and robot config file
    // Otherwise, do nothing
    if (res.success) {
        // Update PID gains through PID managers
        for (const custom_msgs::PIDGain &pid_gain_update : req.pid_gains) {
            PIDLoopTypesEnum loop = static_cast<PIDLoopTypesEnum>(pid_gain_update.loop);
            AxesEnum axis = static_cast<AxesEnum>(pid_gain_update.axis);
            PIDGainTypesEnum gain_type = static_cast<PIDGainTypesEnum>(pid_gain_update.gain);
            pid_managers.at(loop).set_pid_gain(axis, gain_type, pid_gain_update.value);
        }

        // Get all PID gains, including those that were updated and those that were not updated
        LoopsMap<AxesMap<PIDGainsMap>> loops_axes_pid_gains;
        for (const PIDLoopTypesEnum &loop : PID_LOOP_TYPES)
            loops_axes_pid_gains[loop] = pid_managers.at(loop).get_axes_pid_gains();

        // Update robot config file with updated PID gains
        // Throws an exception if file could not be updated successfully; will shut down the node
        ControlsUtils::update_robot_config_pid_gains(loops_axes_pid_gains, cascaded_pid);
    }

    res.message = res.success ? "Updated PID gains successfully."
                              : "Failed to update PID gains. One or more PID gains was invalid.";

    return true;
}

bool Controls::reset_pid_loops_callback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    // Reset all PID loops
    for (const PIDLoopTypesEnum &loop : PID_LOOP_TYPES) pid_managers[loop].reset();

    res.success = true;
    res.message = "Reset PID loops successfully.";
    return true;
}

bool Controls::set_static_power_global_callback(custom_msgs::SetStaticPower::Request &req,
                                                custom_msgs::SetStaticPower::Response &res) {
    tf2::fromMsg(req.static_power, static_power_global);

    // Update static power in robot config file
    // Throws an exception if file could not be updated successfully; will shut down the node
    ControlsUtils::update_robot_config_static_power_global(static_power_global);

    res.success = true;
    res.message = "Updated static power successfully.";

    return true;
}

bool Controls::set_power_scale_factor_callback(custom_msgs::SetPowerScaleFactor::Request &req,
                                               custom_msgs::SetPowerScaleFactor::Response &res) {
    power_scale_factor = req.power_scale_factor;

    // Update power scale factor in robot config file
    // Throws an exception if file could not be updated successfully; will shut down the node
    ControlsUtils::update_robot_config_power_scale_factor(power_scale_factor);

    res.success = true;
    res.message = "Updated power scale factor successfully.";

    return true;
}

void Controls::run() {
    ros::Rate rate(THRUSTER_ALLOCS_RATE);

    Eigen::VectorXd base_power(AXES_COUNT);
    Eigen::VectorXd set_power_unscaled(AXES_COUNT);
    Eigen::VectorXd set_power(AXES_COUNT);
    Eigen::VectorXd unconstrained_allocs;
    Eigen::VectorXd constrained_allocs;
    Eigen::VectorXd actual_power;
    Eigen::VectorXd power_disparity;

    LoopsMap<AxesMap<PIDGainsMap>> loops_axes_pid_gains;

    custom_msgs::ThrusterAllocs constrained_allocs_msg;
    custom_msgs::ThrusterAllocs unconstrained_allocs_msg;
    geometry_msgs::Twist base_power_msg;
    geometry_msgs::Twist set_power_unscaled_msg;
    geometry_msgs::Twist set_power_msg;
    geometry_msgs::Twist actual_power_msg;
    geometry_msgs::Twist power_disparity_msg;
    std_msgs::Float64 power_disparity_norm_msg;
    custom_msgs::ControlTypes control_types_msg;
    std_msgs::Bool status_msg;
    custom_msgs::PIDGains pid_gains_msg;
    geometry_msgs::Vector3 static_power_global_msg;
    std_msgs::Float64 power_scale_factor_msg;

    // Loop while ROS node is running
    while (ros::ok()) {
        // Get set power based on control types
        for (int i = 0; i < AXES_COUNT; i++) {
            switch (control_types.at(AXES[i])) {
                case custom_msgs::ControlTypes::DESIRED_POSITION:
                    // If cascaded PID is enabled, then use velocity PID outputs as set power for DESIRED_POSITION
                    // control type
                    base_power[i] =
                        (cascaded_pid) ? velocity_pid_outputs.at(AXES[i]) : position_pid_outputs.at(AXES[i]);
                    break;
                case custom_msgs::ControlTypes::DESIRED_VELOCITY:
                    base_power[i] = velocity_pid_outputs.at(AXES[i]);
                    break;
                case custom_msgs::ControlTypes::DESIRED_POWER:
                    base_power[i] = desired_power.at(AXES[i]);
                    break;
            }
        }

        // Convert static power local to map
        AxesMap<double> static_power_local_map;
        ControlsUtils::tf_linear_vector_to_map(static_power_local, static_power_local_map);

        // Sum static power local and base power to get set power unscaled
        for (const AxesEnum &axis : AXES) set_power_unscaled[axis] = base_power[axis] + static_power_local_map.at(axis);

        // Apply power scale factor to set power
        set_power = set_power_unscaled * power_scale_factor;

        // Allocate thrusters
        thruster_allocator.allocate_thrusters(set_power, unconstrained_allocs, constrained_allocs, actual_power,
                                              power_disparity);

        // Convert thruster allocation vector to message
        ControlsUtils::eigen_vector_to_thruster_allocs_msg(constrained_allocs, constrained_allocs_msg);

        // Publish thruster allocs if controls are enabled
        if (controls_enabled) thruster_allocs_pub.publish(constrained_allocs_msg);

        // Save actual power to map
        ControlsUtils::eigen_vector_to_map(actual_power, actual_power_map);

        // Publish all other messages
        constrained_thruster_allocs_pub.publish(constrained_allocs_msg);

        ControlsUtils::eigen_vector_to_thruster_allocs_msg(unconstrained_allocs, unconstrained_allocs_msg);
        unconstrained_thruster_allocs_pub.publish(unconstrained_allocs_msg);

        ControlsUtils::eigen_vector_to_twist(base_power, base_power_msg);
        base_power_pub.publish(base_power_msg);

        ControlsUtils::eigen_vector_to_twist(set_power_unscaled, set_power_unscaled_msg);
        set_power_unscaled_pub.publish(set_power_unscaled_msg);

        ControlsUtils::eigen_vector_to_twist(set_power, set_power_msg);
        set_power_pub.publish(set_power_msg);

        ControlsUtils::eigen_vector_to_twist(actual_power, actual_power_msg);
        actual_power_pub.publish(actual_power_msg);

        ControlsUtils::eigen_vector_to_twist(power_disparity, power_disparity_msg);
        power_disparity_pub.publish(power_disparity_msg);

        power_disparity_norm_msg.data = power_disparity.norm();
        power_disparity_norm_pub.publish(power_disparity_norm_msg);

        ControlsUtils::map_to_control_types(control_types, control_types_msg);
        control_types_pub.publish(control_types_msg);

        status_msg.data = controls_enabled;
        status_pub.publish(status_msg);

        for (const PIDLoopTypesEnum &loop : PID_LOOP_TYPES)
            loops_axes_pid_gains[loop] = pid_managers.at(loop).get_axes_pid_gains();

        ControlsUtils::pid_loops_axes_gains_map_to_msg(loops_axes_pid_gains, pid_gains_msg);
        pid_gains_pub.publish(pid_gains_msg);

        static_power_global_msg = tf2::toMsg(static_power_global);
        static_power_global_pub.publish(static_power_global_msg);

        power_scale_factor_msg.data = power_scale_factor;
        power_scale_factor_pub.publish(power_scale_factor_msg);

        // Spin once to let ROS process messages
        ros::spinOnce();

        // Sleep for a short duration to maintain a consistent loop rate
        rate.sleep();
    }
}

int main(int argc, char **argv) {
    // Initialize ROS node
    ros::init(argc, argv, "controls");

    ros::NodeHandle nh;

    // Create transform buffer and listener
    // Listener will listen for transforms and store them in the buffer
    std::unique_ptr<tf2_ros::Buffer> tf_buffer(new tf2_ros::Buffer());
    std::unique_ptr<tf2_ros::TransformListener> tf_listener(new tf2_ros::TransformListener(*tf_buffer));

    // Instantiate controls object and run
    Controls controls = Controls(argc, argv, nh, std::move(tf_buffer));
    controls.run();

    return 0;
}
