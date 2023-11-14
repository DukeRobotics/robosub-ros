#ifndef CONTROLS_UTILS_H
#define CONTROLS_UTILS_H

#include <map>
#include <string>
#include <geometry_msgs/Twist.h>
#include <custom_msgs/ControlTypes.h>

enum ControlTypesEnum
{
  DESIRED_POSE = custom_msgs::ControlTypes::DESIRED_POSE,
  DESIRED_TWIST = custom_msgs::ControlTypes::DESIRED_TWIST,
  DESIRED_POWER = custom_msgs::ControlTypes::DESIRED_POWER
};

const int AXES_COUNT = 6;
const std::string AXES[AXES_COUNT] = {"x", "y", "z", "roll", "pitch", "yaw"};

class ControlsUtils
{
    public:
        static bool value_in_control_types_enum(uint8_t value);
        static void twist_to_map(const geometry_msgs::Twist *twist, std::map<std::string, double> *map);
};

#endif