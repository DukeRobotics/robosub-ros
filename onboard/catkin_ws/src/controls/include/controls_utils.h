#ifndef CONTROLS_UTILS_H
#define CONTROLS_UTILS_H

#include <cstdlib>
#include <memory>
#include <unordered_map>
#include <string>
#include <geometry_msgs/Twist.h>
#include <custom_msgs/ControlTypes.h>
#include <custom_msgs/PIDGain.h>
#include <Eigen/Dense>

enum ControlTypesEnum : uint8_t
{
    DESIRED_POSE = custom_msgs::ControlTypes::DESIRED_POSE,
    DESIRED_TWIST = custom_msgs::ControlTypes::DESIRED_TWIST,
    DESIRED_POWER = custom_msgs::ControlTypes::DESIRED_POWER
};

enum AxisEnum : uint8_t
{
    X = custom_msgs::PIDGain::AXIS_X,
    Y = custom_msgs::PIDGain::AXIS_Y,
    Z = custom_msgs::PIDGain::AXIS_Z,
    ROLL = custom_msgs::PIDGain::AXIS_ROLL,
    PITCH = custom_msgs::PIDGain::AXIS_PITCH,
    YAW = custom_msgs::PIDGain::AXIS_YAW
};
const int AXES_COUNT = 6;
const AxisEnum AXES[AXES_COUNT] = {AxisEnum::X, AxisEnum::Y, AxisEnum::Z,
                                   AxisEnum::ROLL, AxisEnum::PITCH, AxisEnum::YAW};

const std::unordered_map<AxisEnum, std::string> AXES_NAMES = {
    {AxisEnum::X, "x"},
    {AxisEnum::Y, "y"},
    {AxisEnum::Z, "z"},
    {AxisEnum::ROLL, "roll"},
    {AxisEnum::PITCH, "pitch"},
    {AxisEnum::YAW, "yaw"}};

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

const std::string ROBOT_CONFIG_FILE_PATH = "/root/dev/robosub-ros/onboard/catkin_ws/src/controls/config/" + std::string(std::getenv("ROBOT_NAME")) + ".yaml";

typedef std::unordered_map<PIDGainTypesEnum, double> PIDGainsMap;
typedef std::unordered_map<AxisEnum, std::shared_ptr<PIDGainsMap>> AxesPIDGainsMap;
typedef std::unordered_map<PIDLoopTypesEnum, AxesPIDGainsMap> LoopsAxesPIDGainsMap;

class ControlsUtils
{
public:
    static bool value_in_control_types_enum(uint8_t value);
    static void twist_to_map(const geometry_msgs::Twist &twist, std::unordered_map<AxisEnum, double> &map);
    static void eigen_vector_to_twist(const Eigen::VectorXd &vector, geometry_msgs::Twist &twist);
    static bool control_types_to_map(const custom_msgs::ControlTypes &control_types,
                                     std::unordered_map<AxisEnum, ControlTypesEnum> &map);
    static void map_to_control_types(const std::unordered_map<AxisEnum, ControlTypesEnum> &map,
                                     custom_msgs::ControlTypes &control_types);
    static void read_matrix_from_csv(std::string file_path, Eigen::MatrixXd &matrix);
    static void read_robot_config(std::string file_path, LoopsAxesPIDGainsMap &robot_config, std::string &wrench_matrix_file_path, std::string &wrench_matrix_pinv_file_path);
};

#endif