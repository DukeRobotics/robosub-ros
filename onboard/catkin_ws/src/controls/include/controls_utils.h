#ifndef CONTROLS_UTILS_H
#define CONTROLS_UTILS_H

#include <unordered_map>
#include <string>
#include <geometry_msgs/Twist.h>
#include <custom_msgs/ControlTypes.h>
#include <Eigen/Dense>

enum ControlTypesEnum
{
    DESIRED_POSE = custom_msgs::ControlTypes::DESIRED_POSE,
    DESIRED_TWIST = custom_msgs::ControlTypes::DESIRED_TWIST,
    DESIRED_POWER = custom_msgs::ControlTypes::DESIRED_POWER
};

enum AxisEnum
{
    X = 0,
    Y = 1,
    Z = 2,
    ROLL = 3,
    PITCH = 4,
    YAW = 5
};
const int AXES_COUNT = 6;
const AxisEnum AXES[AXES_COUNT] = {AxisEnum::X, AxisEnum::Y, AxisEnum::Z,
                                   AxisEnum::ROLL, AxisEnum::PITCH, AxisEnum::YAW};

enum PIDLoopTypesEnum
{
    POSITION = 0,
    VELOCITY = 1
};
const int PID_LOOP_TYPES_COUNT = 2;
const PIDLoopTypesEnum PID_LOOP_TYPES[PID_LOOP_TYPES_COUNT] = {PIDLoopTypesEnum::POSITION, PIDLoopTypesEnum::VELOCITY};

enum PIDGainTypesEnum
{
    KP = 0,
    KI = 1,
    KD = 2,
    FF = 3
};
const int PID_GAIN_TYPES_COUNT = 4;
const PIDGainTypesEnum PID_GAIN_TYPES[PID_GAIN_TYPES_COUNT] = {PIDGainTypesEnum::KP, PIDGainTypesEnum::KI,
                                                               PIDGainTypesEnum::KD, PIDGainTypesEnum::FF};

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
};

#endif