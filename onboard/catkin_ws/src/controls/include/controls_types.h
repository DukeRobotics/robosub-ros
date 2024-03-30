#ifndef CONTROLS_TYPES_H
#define CONTROLS_TYPES_H

#include <custom_msgs/ControlTypes.h>
#include <custom_msgs/PIDDerivativeType.h>
#include <custom_msgs/PIDGain.h>
#include <ros/package.h>

#include <unordered_map>

// Control types: DESIRED_POSITION, DESIRED_VELOCITY, DESIRED_POWER
enum ControlTypesEnum : uint8_t {
    DESIRED_POSITION = custom_msgs::ControlTypes::DESIRED_POSITION,
    DESIRED_VELOCITY = custom_msgs::ControlTypes::DESIRED_VELOCITY,
    DESIRED_POWER = custom_msgs::ControlTypes::DESIRED_POWER
};

// Number of control types
const int CONTROL_TYPES_COUNT = 3;

// Control types array
const ControlTypesEnum CONTROL_TYPES[CONTROL_TYPES_COUNT] = {
    ControlTypesEnum::DESIRED_POSITION, ControlTypesEnum::DESIRED_VELOCITY, ControlTypesEnum::DESIRED_POWER};

// Axes: X, Y, Z, ROLL, PITCH, YAW
enum AxesEnum : uint8_t {
    X = custom_msgs::PIDGain::AXIS_X,
    Y = custom_msgs::PIDGain::AXIS_Y,
    Z = custom_msgs::PIDGain::AXIS_Z,
    ROLL = custom_msgs::PIDGain::AXIS_ROLL,
    PITCH = custom_msgs::PIDGain::AXIS_PITCH,
    YAW = custom_msgs::PIDGain::AXIS_YAW
};

// Number of axes
const int AXES_COUNT = 6;

// Axes array
const AxesEnum AXES[AXES_COUNT] = {AxesEnum::X,    AxesEnum::Y,     AxesEnum::Z,
                                   AxesEnum::ROLL, AxesEnum::PITCH, AxesEnum::YAW};

// Axes string names
const std::unordered_map<AxesEnum, std::string> AXES_NAMES = {{AxesEnum::X, "x"},         {AxesEnum::Y, "y"},
                                                              {AxesEnum::Z, "z"},         {AxesEnum::ROLL, "roll"},
                                                              {AxesEnum::PITCH, "pitch"}, {AxesEnum::YAW, "yaw"}};

// PID loop types: POSITION, VELOCITY
enum PIDLoopTypesEnum : uint8_t {
    POSITION = custom_msgs::PIDGain::LOOP_POSITION,
    VELOCITY = custom_msgs::PIDGain::LOOP_VELOCITY
};

// Number of PID loop types
const int PID_LOOP_TYPES_COUNT = 2;

// PID loop types array
const PIDLoopTypesEnum PID_LOOP_TYPES[PID_LOOP_TYPES_COUNT] = {PIDLoopTypesEnum::POSITION, PIDLoopTypesEnum::VELOCITY};

// PID loop types string names
const std::unordered_map<PIDLoopTypesEnum, std::string> PID_LOOP_TYPES_NAMES = {
    {PIDLoopTypesEnum::POSITION, "position"}, {PIDLoopTypesEnum::VELOCITY, "velocity"}};

// Name of cascaded position PID loop in robot config file
const std::string CASCADED_POSITION_PID_NAME = "position_cascaded";

// PID gain types: KP, KI, KD, FF
enum PIDGainTypesEnum : uint8_t {
    KP = custom_msgs::PIDGain::GAIN_KP,
    KI = custom_msgs::PIDGain::GAIN_KI,
    KD = custom_msgs::PIDGain::GAIN_KD,
    FF = custom_msgs::PIDGain::GAIN_FF
};

// Number of PID gain types
const int PID_GAIN_TYPES_COUNT = 4;

// PID gain types array
const PIDGainTypesEnum PID_GAIN_TYPES[PID_GAIN_TYPES_COUNT] = {PIDGainTypesEnum::KP, PIDGainTypesEnum::KI,
                                                               PIDGainTypesEnum::KD, PIDGainTypesEnum::FF};

// PID gain types string names
const std::unordered_map<PIDGainTypesEnum, std::string> PID_GAIN_TYPES_NAMES = {{PIDGainTypesEnum::KP, "Kp"},
                                                                                {PIDGainTypesEnum::KI, "Ki"},
                                                                                {PIDGainTypesEnum::KD, "Kd"},
                                                                                {PIDGainTypesEnum::FF, "Ff"}};

// Path to controls package
const std::string CONTROLS_PACKAGE_PATH = ros::package::getPath("controls");

// Path to robot config file
const std::string ROBOT_CONFIG_FILE_PATH =
    CONTROLS_PACKAGE_PATH + "/config/" + std::string(std::getenv("ROBOT_NAME") ? std::getenv("ROBOT_NAME") : "oogway") +
    ".yaml";

// Map of PID gains
typedef std::unordered_map<PIDGainTypesEnum, double> PIDGainsMap;

// Map of axes to template type
template <typename T>
using AxesMap = std::unordered_map<AxesEnum, T>;

// Map of PID loop types to axes to template type
template <typename T>
using LoopsMap = std::unordered_map<PIDLoopTypesEnum, T>;

// PID Derivative Types: CALCULATED, PROVIDED
enum PIDDerivativeTypesEnum : uint8_t {
    CALCULATED = custom_msgs::PIDDerivativeType::CALCULATED,
    PROVIDED = custom_msgs::PIDDerivativeType::PROVIDED
};

// Number of PID derivative types
const int PID_DERIVATIVE_TYPES_COUNT = 2;

// PID derivative types array
const PIDDerivativeTypesEnum PID_DERIVATIVE_TYPES[PID_DERIVATIVE_TYPES_COUNT] = {PIDDerivativeTypesEnum::CALCULATED,
                                                                                 PIDDerivativeTypesEnum::PROVIDED};

// PID terms that are summed to get control effort
struct PIDTerms {
    double proportional;
    double integral;
    double derivative;
    double feedforward;
};

// Values computed by PID loop
struct PIDInfo {
    PIDTerms terms;

    double filtered_error;
    double integral;
    double filtered_derivative;

    double calculated_derivative;
    double provided_derivative;
    PIDDerivativeTypesEnum derivative_type;
};

#endif