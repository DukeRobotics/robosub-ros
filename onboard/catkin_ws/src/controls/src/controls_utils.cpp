#include <map>
#include <string>
#include <geometry_msgs/Twist.h>
#include <custom_msgs/ControlTypes.h>

#include "controls_utils.h"

bool ControlsUtils::value_in_control_types_enum(uint8_t value)
{
    return value == ControlTypesEnum::DESIRED_POSE ||
           value == ControlTypesEnum::DESIRED_TWIST ||
           value == ControlTypesEnum::DESIRED_POWER;
}

void ControlsUtils::twist_to_map(const geometry_msgs::Twist *twist, std::map<std::string, double> *map)
{
    (*map)[AXES[0]] = twist->linear.x;
    (*map)[AXES[1]] = twist->linear.y;
    (*map)[AXES[2]] = twist->linear.z;
    (*map)[AXES[3]] = twist->angular.x;
    (*map)[AXES[4]] = twist->angular.y;
    (*map)[AXES[5]] = twist->angular.z;
}