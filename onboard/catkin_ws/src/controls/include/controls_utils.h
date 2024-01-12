#ifndef CONTROLS_UTILS_H
#define CONTROLS_UTILS_H

#include <cstdlib>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <string>
#include <vector>
#include <ros/package.h>
#include <tf2/LinearMath/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <custom_msgs/ControlTypes.h>
#include <custom_msgs/PIDGain.h>
#include <custom_msgs/PIDGains.h>
#include <custom_msgs/ThrusterAllocs.h>
#include <Eigen/Dense>

enum ControlTypesEnum : uint8_t
{
    DESIRED_POSE = custom_msgs::ControlTypes::DESIRED_POSE,
    DESIRED_TWIST = custom_msgs::ControlTypes::DESIRED_TWIST,
    DESIRED_POWER = custom_msgs::ControlTypes::DESIRED_POWER
};

enum AxesEnum : uint8_t
{
    X = custom_msgs::PIDGain::AXIS_X,
    Y = custom_msgs::PIDGain::AXIS_Y,
    Z = custom_msgs::PIDGain::AXIS_Z,
    ROLL = custom_msgs::PIDGain::AXIS_ROLL,
    PITCH = custom_msgs::PIDGain::AXIS_PITCH,
    YAW = custom_msgs::PIDGain::AXIS_YAW
};
const int AXES_COUNT = 6;
const AxesEnum AXES[AXES_COUNT] = {AxesEnum::X, AxesEnum::Y, AxesEnum::Z,
                                   AxesEnum::ROLL, AxesEnum::PITCH, AxesEnum::YAW};

const std::unordered_map<AxesEnum, std::string> AXES_NAMES = {
    {AxesEnum::X, "x"},
    {AxesEnum::Y, "y"},
    {AxesEnum::Z, "z"},
    {AxesEnum::ROLL, "roll"},
    {AxesEnum::PITCH, "pitch"},
    {AxesEnum::YAW, "yaw"}};

enum PIDLoopTypesEnum : uint8_t
{
    POSITION = custom_msgs::PIDGain::LOOP_POSITION,
    VELOCITY = custom_msgs::PIDGain::LOOP_VELOCITY
};
const int PID_LOOP_TYPES_COUNT = 2;
const PIDLoopTypesEnum PID_LOOP_TYPES[PID_LOOP_TYPES_COUNT] = {PIDLoopTypesEnum::POSITION, PIDLoopTypesEnum::VELOCITY};
const std::unordered_map<PIDLoopTypesEnum, std::string> PID_LOOP_TYPES_NAMES = {
    {PIDLoopTypesEnum::POSITION, "position"},
    {PIDLoopTypesEnum::VELOCITY, "velocity"}};

enum PIDGainTypesEnum : uint8_t
{
    KP = custom_msgs::PIDGain::GAIN_KP,
    KI = custom_msgs::PIDGain::GAIN_KI,
    KD = custom_msgs::PIDGain::GAIN_KD,
    FF = custom_msgs::PIDGain::GAIN_FF
};
const int PID_GAIN_TYPES_COUNT = 4;
const PIDGainTypesEnum PID_GAIN_TYPES[PID_GAIN_TYPES_COUNT] = {PIDGainTypesEnum::KP, PIDGainTypesEnum::KI,
                                                               PIDGainTypesEnum::KD, PIDGainTypesEnum::FF};
const std::unordered_map<PIDGainTypesEnum, std::string> PID_GAIN_TYPES_NAMES = {
    {PIDGainTypesEnum::KP, "Kp"},
    {PIDGainTypesEnum::KI, "Ki"},
    {PIDGainTypesEnum::KD, "Kd"},
    {PIDGainTypesEnum::FF, "Ff"}};

const std::string CONTROLS_PACKAGE_PATH = ros::package::getPath("controls");
const std::string ROBOT_CONFIG_FILE_PATH = CONTROLS_PACKAGE_PATH + "/config/" + std::string(std::getenv("ROBOT_NAME")) + ".yaml";

typedef std::unordered_map<PIDGainTypesEnum, double> PIDGainsMap;
typedef std::unordered_map<AxesEnum, std::shared_ptr<PIDGainsMap>> AxesPIDGainsMap;
typedef std::unordered_map<PIDLoopTypesEnum, AxesPIDGainsMap> LoopsAxesPIDGainsMap;

namespace ControlsUtils
{
    extern std::mutex robot_config_mutex;

    bool value_in_control_types_enum(uint8_t value);
    bool value_in_axes_enum(uint8_t value);
    bool value_in_pid_loop_types_enum(uint8_t value);
    bool value_in_pid_gain_types_enum(uint8_t value);
    bool quaternion_valid(const geometry_msgs::Quaternion &quaternion);
    bool twist_in_range(const geometry_msgs::Twist &twist, double min, double max);
    bool pid_gain_valid(const custom_msgs::PIDGain &pid_gain);
    bool pid_gains_valid(const std::vector<custom_msgs::PIDGain> &pid_gains);
    void pose_to_twist(const geometry_msgs::Pose &pose, geometry_msgs::Twist &twist);
    void twist_to_map(const geometry_msgs::Twist &twist, std::unordered_map<AxesEnum, double> &map);
    void map_to_twist(const std::unordered_map<AxesEnum, double> &map, geometry_msgs::Twist &twist);
    void eigen_vector_to_thruster_allocs(const Eigen::VectorXd &vector, custom_msgs::ThrusterAllocs &thruster_allocs);
    void eigen_vector_to_twist(const Eigen::VectorXd &vector, geometry_msgs::Twist &twist);
    bool control_types_to_map(const custom_msgs::ControlTypes &control_types,
                                     std::unordered_map<AxesEnum, ControlTypesEnum> &map);
    void map_to_control_types(const std::unordered_map<AxesEnum, ControlTypesEnum> &map,
                                     custom_msgs::ControlTypes &control_types);
    void tf_linear_vector_to_map(const tf2::Vector3 &vector, std::unordered_map<AxesEnum, double> &map);
    bool update_pid_loops_axes_gains_map(LoopsAxesPIDGainsMap &all_pid_gains,
                                                const std::vector<custom_msgs::PIDGain> &pid_gain_updates);
    void pid_loops_axes_gains_map_to_msg(const LoopsAxesPIDGainsMap &all_pid_gains,
                                                custom_msgs::PIDGains &pid_gains_msg);
    void populate_axes_map(std::unordered_map<AxesEnum, double> &map, double value);
    void read_matrix_from_csv(std::string file_path, Eigen::MatrixXd &matrix);
    void read_robot_config(std::string file_path,
                                  LoopsAxesPIDGainsMap &all_pid_gains,
                                  tf2::Vector3 &static_power_global,
                                  double &power_scale_factor,
                                  std::string &wrench_matrix_file_path,
                                  std::string &wrench_matrix_pinv_file_path);
    void update_robot_pid_gains(std::string file_path, const LoopsAxesPIDGainsMap &all_pid_gains);
    void update_robot_static_power_global(std::string file_path, const tf2::Vector3 &static_power_global);
    void update_robot_power_scale_factor(std::string file_path, double &power_scale_factor);
};

#endif