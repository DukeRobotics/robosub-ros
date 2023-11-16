#include <unordered_map>
#include <string>
#include <geometry_msgs/Twist.h>
#include <custom_msgs/ControlTypes.h>
#include <Eigen/Dense>

#include "controls_utils.h"

bool ControlsUtils::value_in_control_types_enum(uint8_t value)
{
    return value == ControlTypesEnum::DESIRED_POSE ||
           value == ControlTypesEnum::DESIRED_TWIST ||
           value == ControlTypesEnum::DESIRED_POWER;
}

void ControlsUtils::twist_to_map(const geometry_msgs::Twist &twist, std::unordered_map<AxisEnum, double> &map)
{
    map[AxisEnum::X] = twist.linear.x;
    map[AxisEnum::Y] = twist.linear.y;
    map[AxisEnum::Z] = twist.linear.z;
    map[AxisEnum::ROLL] = twist.angular.x;
    map[AxisEnum::PITCH] = twist.angular.y;
    map[AxisEnum::YAW] = twist.angular.z;
}

void ControlsUtils::eigen_vector_to_twist(const Eigen::VectorXd &vector, geometry_msgs::Twist &twist)
{
    twist.linear.x = vector(0);
    twist.linear.y = vector(1);
    twist.linear.z = vector(2);
    twist.angular.x = vector(3);
    twist.angular.y = vector(4);
    twist.angular.z = vector(5);
}

bool ControlsUtils::control_types_to_map(const custom_msgs::ControlTypes &control_types,
                                         std::unordered_map<AxisEnum, ControlTypesEnum> &map)
{
    std::unordered_map<AxisEnum, uint8_t> new_control_types;
    new_control_types[AxisEnum::X] = control_types.x;
    new_control_types[AxisEnum::Y] = control_types.y;
    new_control_types[AxisEnum::Z] = control_types.z;
    new_control_types[AxisEnum::ROLL] = control_types.roll;
    new_control_types[AxisEnum::PITCH] = control_types.pitch;
    new_control_types[AxisEnum::YAW] = control_types.yaw;

    for (const auto& pair : new_control_types)
        if (!value_in_control_types_enum(pair.second))
            return false;

    for (const auto& pair : new_control_types)
        map[pair.first] = static_cast<ControlTypesEnum>(pair.second);

    return true;
}

void ControlsUtils::map_to_control_types(const std::unordered_map<AxisEnum, ControlTypesEnum> &map,
                                         custom_msgs::ControlTypes &control_types)
{
    control_types.x = static_cast<uint8_t>(map.at(AxisEnum::X));
    control_types.y = static_cast<uint8_t>(map.at(AxisEnum::Y));
    control_types.z = static_cast<uint8_t>(map.at(AxisEnum::Z));
    control_types.roll = static_cast<uint8_t>(map.at(AxisEnum::ROLL));
    control_types.pitch = static_cast<uint8_t>(map.at(AxisEnum::PITCH));
    control_types.yaw = static_cast<uint8_t>(map.at(AxisEnum::YAW));
}