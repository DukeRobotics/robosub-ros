#ifndef CONTROLS_H
#define CONTROLS_H

#include <custom_msgs/ControlTypes.h>
#include <custom_msgs/PIDGain.h>
#include <custom_msgs/PIDGains.h>
#include <custom_msgs/SetControlTypes.h>
#include <custom_msgs/SetPIDGains.h>
#include <custom_msgs/SetPowerScaleFactor.h>
#include <custom_msgs/SetStaticPower.h>
#include <custom_msgs/ThrusterAllocs.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
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

class Controls {
   private:
    // Rate at which thruster allocations are published (Hz)
    static const int THRUSTER_ALLOCS_RATE;

    // Launch file parameters
    bool sim;
    bool enable_position_pid;
    bool enable_velocity_pid;
    bool cascaded_pid;

    // Transform buffer
    // Unique pointer is used to avoid writing a custom constructor and destructor for this class
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;

    // Whether controls are enabled
    bool controls_enabled;

    // Control type to use for each axis
    AxesMap<ControlTypesEnum> control_types;

    // Desired states
    geometry_msgs::Pose desired_position;
    geometry_msgs::Twist desired_velocity;
    AxesMap<double> desired_power;

    // Current state
    nav_msgs::Odometry state;

    // Timestamp of last state message
    ros::Time last_state_msg_time;

    // Managers for both position and velocity PID loops
    LoopsMap<PIDManager> pid_managers;

    // Control efforts output by PID loops
    AxesMap<double> position_pid_outputs;
    AxesMap<double> velocity_pid_outputs;

    // Values computed by PID loops
    AxesMap<PIDInfo> position_pid_infos;
    AxesMap<PIDInfo> velocity_pid_infos;

    // Thruster allocator
    ThrusterAllocator thruster_allocator;

    // Minimum and maximum desired power for each axis
    AxesMap<double> desired_power_min;
    AxesMap<double> desired_power_max;

    // Static power (to offset buoyancy and other persistent forces)
    tf2::Vector3 static_power_global;
    tf2::Vector3 static_power_local;

    // Constant multiplier for set power
    double power_scale_factor;

    // Most recent actual power
    AxesMap<double> actual_power_map;

    // ROS topic subscribers
    ros::Subscriber state_sub;
    ros::Subscriber desired_position_sub;
    ros::Subscriber desired_velocity_sub;
    ros::Subscriber desired_power_sub;

    // ROS service advertisers
    ros::ServiceServer enable_controls_srv;
    ros::ServiceServer set_control_types_srv;
    ros::ServiceServer set_pid_gains_srv;
    ros::ServiceServer reset_pid_loops_srv;
    ros::ServiceServer set_static_power_global_srv;
    ros::ServiceServer set_power_scale_factor_srv;

    // ROS topic publishers
    ros::Publisher thruster_allocs_pub;
    ros::Publisher constrained_thruster_allocs_pub;
    ros::Publisher unconstrained_thruster_allocs_pub;
    ros::Publisher base_power_pub;
    ros::Publisher set_power_unscaled_pub;
    ros::Publisher set_power_pub;
    ros::Publisher actual_power_pub;
    ros::Publisher power_disparity_pub;
    ros::Publisher power_disparity_norm_pub;
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

    // *****************************************************************************************************************
    // ROS subscriber callbacks

    /**
     * @brief Callback for desired position messages.
     *
     * @param msg Desired position message.
     *
     * @note The orientation of the desired position must be a valid unit quaternion.
     */
    void desired_position_callback(const geometry_msgs::Pose msg);

    /**
     * @brief Callback for desired velocity messages.
     *
     * @param msg Desired velocity message.
     */
    void desired_velocity_callback(const geometry_msgs::Twist msg);

    /**
     * @brief Callback for desired power messages.
     *
     * @param msg Desired power message.
     *
     * @note The value of each axis in the desired power message must be in the range [-1, 1].
     */
    void desired_power_callback(const geometry_msgs::Twist msg);

    /**
     * @brief Callback for state messages. Runs PID loops.
     *
     * @param msg State message.
     */
    void state_callback(const nav_msgs::Odometry msg);

    // *****************************************************************************************************************
    // ROS service callbacks

    /**
     * @brief Callback for enabling/disabling controls.
     *
     * @param req Request indicating whether to enable/disable controls.
     * @param res Response indicating whether controls were enabled/disabled.
     * @return True if the service response was successfully filled, false otherwise.
     */
    bool enable_controls_callback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

    /**
     * @brief Callback for setting control types.
     *
     * @param req Request indicating control types to set for each axis.
     * @param res Response indicating whether control types were set.
     * @return True if the service response was successfully filled, false otherwise.
     */
    bool set_control_types_callback(custom_msgs::SetControlTypes::Request &req,
                                    custom_msgs::SetControlTypes::Response &res);

    /**
     * @brief Callback for updating PID gains.
     *
     * @param req Request providing updated PID gains.
     * @param res Response indicating whether PID gains were updated.
     * @return True if the service response was successfully filled, false otherwise.
     *
     * @throws ros::Exception Robot config file could not be updated with the new PID gains.
     */
    bool set_pid_gains_callback(custom_msgs::SetPIDGains::Request &req, custom_msgs::SetPIDGains::Response &res);

    /**
     * @brief Callback for resetting PID loops.
     *
     * @param req Request indicating that PID loops should be reset.
     * @param res Response indicating whether PID loops were reset.
     * @return True if the service response was successfully filled, false otherwise.
     */
    bool reset_pid_loops_callback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    /**
     * @brief Callback for updating static power global.
     *
     * @param req Request providing updated static power global.
     * @param res Response indicating whether static power global was updated.
     * @return True if the service response was successfully filled, false otherwise.
     *
     * @throws ros::Exception Robot config file could not be updated with the new static power global.
     */
    bool set_static_power_global_callback(custom_msgs::SetStaticPower::Request &req,
                                          custom_msgs::SetStaticPower::Response &res);

    /**
     * @brief Callback for updating power scale factor.
     *
     * @param req Request providing power scale factor to set.
     * @param res Response indicating whether power scale factor was set.
     * @return True if the service response was successfully filled, false otherwise.
     *
     * @throws ros::Exception Robot config file could not be updated with the new power scale factor.
     */
    bool set_power_scale_factor_callback(custom_msgs::SetPowerScaleFactor::Request &req,
                                         custom_msgs::SetPowerScaleFactor::Response &res);

   public:
    /**
     * @brief Construct a new Controls object. Initializes PID loops, thruster allocator, ROS subscribers, service
     *  advertisers, and publishers.
     *
     * @param argc Number of command line arguments.
     * @param argv Command line arguments.
     * @param nh ROS node handle. Used to create ROS subscribers, service advertisers, and publishers.
     * @param tf_buffer Transform buffer. Used to obtain transforms between frames.
     */
    Controls(int argc, char **argv, ros::NodeHandle &nh, std::unique_ptr<tf2_ros::Buffer> tf_buffer);

    /**
     * @brief Loop that runs while the node is active. Allocates thrusters based on control efforts and publishes
     *  thruster allocations, along with other diagnostic information.
     */
    void run();
};

#endif