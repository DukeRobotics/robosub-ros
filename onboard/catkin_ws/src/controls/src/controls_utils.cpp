#include <iostream>
#include <fstream>
#include <sstream>
#include <memory>
#include <unordered_map>
#include <string>
#include <ros/ros.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <custom_msgs/ControlTypes.h>
#include <custom_msgs/PIDGain.h>
#include <custom_msgs/PIDGains.h>
#include <custom_msgs/ThrusterAllocs.h>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include "controls_utils.h"

bool ControlsUtils::value_in_control_types_enum(uint8_t value)
{
    return value == ControlTypesEnum::DESIRED_POSE ||
           value == ControlTypesEnum::DESIRED_TWIST ||
           value == ControlTypesEnum::DESIRED_POWER;
}

bool ControlsUtils::value_in_axes_enum(uint8_t value)
{
    return value == AxesEnum::X ||
           value == AxesEnum::Y ||
           value == AxesEnum::Z ||
           value == AxesEnum::ROLL ||
           value == AxesEnum::PITCH ||
           value == AxesEnum::YAW;
}

bool ControlsUtils::value_in_pid_loop_types_enum(uint8_t value)
{
    return value == PIDLoopTypesEnum::POSITION ||
           value == PIDLoopTypesEnum::VELOCITY;
}

bool ControlsUtils::value_in_pid_gain_types_enum(uint8_t value)
{
    return value == PIDGainTypesEnum::KP ||
           value == PIDGainTypesEnum::KI ||
           value == PIDGainTypesEnum::KD ||
           value == PIDGainTypesEnum::FF;
}

bool ControlsUtils::quaternion_valid(const geometry_msgs::Quaternion &quaternion)
{
    // Ensure quaternion has length 1
    // Otherwise it is not a valid rotation
    tf2::Quaternion q;
    tf2::fromMsg(quaternion, q);
    return std::abs(q.length() - 1.0) < 1e-6;
}

bool ControlsUtils::twist_in_range(const geometry_msgs::Twist &twist, double min, double max)
{
    return twist.linear.x >= min && twist.linear.x <= max &&
           twist.linear.y >= min && twist.linear.y <= max &&
           twist.linear.z >= min && twist.linear.z <= max &&
           twist.angular.x >= min && twist.angular.x <= max &&
           twist.angular.y >= min && twist.angular.y <= max &&
           twist.angular.z >= min && twist.angular.z <= max;
}

bool ControlsUtils::pid_gain_valid(const custom_msgs::PIDGain &pid_gain)
{
    return value_in_pid_loop_types_enum(pid_gain.loop) &&
           value_in_axes_enum(pid_gain.axis) &&
           value_in_pid_gain_types_enum(pid_gain.gain);
}

bool ControlsUtils::pid_gains_valid(const std::vector<custom_msgs::PIDGain> &pid_gains)
{
    for (const custom_msgs::PIDGain &pid_gain : pid_gains)
        if (!pid_gain_valid(pid_gain))
            return false;

    return true;
}

void ControlsUtils::pose_to_twist(const geometry_msgs::Pose &pose, geometry_msgs::Twist &twist)
{
    twist.linear.x = pose.position.x;
    twist.linear.y = pose.position.y;
    twist.linear.z = pose.position.z;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = 0.0;

    // Get roll, pitch, yaw from quaternion
    // The order of rotation is roll, pitch, yaw
    // Roll, pitch, yaw are in radians with range [-pi, pi]
    tf2::Quaternion q;
    tf2::fromMsg(pose.orientation, q);
    tf2::Matrix3x3 m(q);
    m.getRPY(twist.angular.x, twist.angular.y, twist.angular.z);
}

void ControlsUtils::twist_to_map(const geometry_msgs::Twist &twist, std::unordered_map<AxesEnum, double> &map)
{
    map[AxesEnum::X] = twist.linear.x;
    map[AxesEnum::Y] = twist.linear.y;
    map[AxesEnum::Z] = twist.linear.z;
    map[AxesEnum::ROLL] = twist.angular.x;
    map[AxesEnum::PITCH] = twist.angular.y;
    map[AxesEnum::YAW] = twist.angular.z;
}

void ControlsUtils::eigen_vector_to_thruster_allocs(const Eigen::VectorXd &vector, custom_msgs::ThrusterAllocs &thruster_allocs)
{
    thruster_allocs.allocs.clear();
    for (int i = 0; i < vector.rows(); ++i)
        thruster_allocs.allocs.push_back(vector(i));
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
                                         std::unordered_map<AxesEnum, ControlTypesEnum> &map)
{
    std::unordered_map<AxesEnum, uint8_t> new_control_types;
    new_control_types[AxesEnum::X] = control_types.x;
    new_control_types[AxesEnum::Y] = control_types.y;
    new_control_types[AxesEnum::Z] = control_types.z;
    new_control_types[AxesEnum::ROLL] = control_types.roll;
    new_control_types[AxesEnum::PITCH] = control_types.pitch;
    new_control_types[AxesEnum::YAW] = control_types.yaw;

    for (const auto &pair : new_control_types)
        if (!value_in_control_types_enum(pair.second))
            return false;

    for (const auto &pair : new_control_types)
        map[pair.first] = static_cast<ControlTypesEnum>(pair.second);

    return true;
}

void ControlsUtils::map_to_control_types(const std::unordered_map<AxesEnum, ControlTypesEnum> &map,
                                         custom_msgs::ControlTypes &control_types)
{
    control_types.x = map.at(AxesEnum::X);
    control_types.y = map.at(AxesEnum::Y);
    control_types.z = map.at(AxesEnum::Z);
    control_types.roll = map.at(AxesEnum::ROLL);
    control_types.pitch = map.at(AxesEnum::PITCH);
    control_types.yaw = map.at(AxesEnum::YAW);
}

void ControlsUtils::tf_linear_vector_to_map(const tf2::Vector3 &vector, std::unordered_map<AxesEnum, double> &map)
{
    map[AxesEnum::X] = vector.getX();
    map[AxesEnum::Y] = vector.getY();
    map[AxesEnum::Z] = vector.getZ();
    map[AxesEnum::ROLL] = 0.0;
    map[AxesEnum::PITCH] = 0.0;
    map[AxesEnum::YAW] = 0.0;
}

bool ControlsUtils::update_pid_loops_axes_gains_map(LoopsAxesPIDGainsMap &all_pid_gains, const std::vector<custom_msgs::PIDGain> &pid_gain_updates)
{
    if (!ControlsUtils::pid_gains_valid(pid_gain_updates))
        return false;

    for (const custom_msgs::PIDGain &pid_gain_update : pid_gain_updates)
    {
        PIDLoopTypesEnum loop = static_cast<PIDLoopTypesEnum>(pid_gain_update.loop);
        AxesEnum axis = static_cast<AxesEnum>(pid_gain_update.axis);
        PIDGainTypesEnum gain = static_cast<PIDGainTypesEnum>(pid_gain_update.gain);
        (*all_pid_gains[loop][axis])[gain] = pid_gain_update.value;
    }

    return true;
}

void ControlsUtils::pid_loops_axes_gains_map_to_msg(const LoopsAxesPIDGainsMap &all_pid_gains, custom_msgs::PIDGains &pid_gains_msg)
{
    pid_gains_msg.pid_gains.clear();

    for (const PIDLoopTypesEnum &loop : PID_LOOP_TYPES)
        for (const AxesEnum &axis : AXES)
            for (const PIDGainTypesEnum &gain : PID_GAIN_TYPES)
            {
                custom_msgs::PIDGain pid_gain;
                pid_gain.loop = loop;
                pid_gain.axis = axis;
                pid_gain.gain = gain;
                pid_gain.value = all_pid_gains.at(loop).at(axis)->at(gain);
                pid_gains_msg.pid_gains.push_back(pid_gain);
            }
}

void ControlsUtils::populate_axes_map(std::unordered_map<AxesEnum, double> &map, double value)
{
    for (const AxesEnum &axis : AXES)
        map[axis] = value;
}

void ControlsUtils::read_matrix_from_csv(std::string file_path, Eigen::MatrixXd &matrix)
{
    // Open the CSV file
    std::ifstream file(file_path);

    ROS_ASSERT_MSG(file.is_open(), "Could not open CSV file. %s", file_path.c_str());

    // Read data from the CSV file and initialize the matrix
    std::vector<std::vector<double>> data;
    std::string line;
    int cols = -1;

    while (std::getline(file, line))
    {
        std::istringstream iss(line);
        std::vector<double> row;

        double value;
        char comma;

        while (iss >> value)
        {
            row.push_back(value);

            iss >> comma;
            ROS_ASSERT_MSG((iss.good() || iss.eof()) && comma == ',', "File is not in valid CSV format. %s", file_path.c_str());
        }

        data.push_back(row);

        // Ensure all rows have the same number of columns
        if (cols == -1)
            cols = row.size();
        else
            ROS_ASSERT_MSG(row.size() == cols, "CSV file must have same number of columns in all rows. %s", file_path.c_str());
    }

    // Close the file
    file.close();

    // Determine the matrix size
    int rows = data.size();

    // Ensure the matrix has at least one row and one column
    ROS_ASSERT_MSG(cols >= 1, "CSV file must have at least one column. %s", file_path.c_str());
    ROS_ASSERT_MSG(rows >= 1, "CSV file must have at least one row. %s", file_path.c_str());

    // Initialize the Eigen matrix
    matrix.resize(rows, cols);

    // Copy the data from the vector of vectors to the Eigen matrix
    for (int i = 0; i < rows; ++i)
        for (int j = 0; j < cols; ++j)
            matrix(i, j) = data[i][j];
}

void ControlsUtils::read_robot_config(std::string file_path,
                                      LoopsAxesPIDGainsMap &all_pid_gains,
                                      tf2::Vector3 &static_power_global,
                                      double &power_multiplier,
                                      std::string &wrench_matrix_file_path,
                                      std::string &wrench_matrix_pinv_file_path)
{
    try
    {
        YAML::Node config = YAML::LoadFile(file_path);

        YAML::Node pid_node = config["pid"];

        for (const auto &loop_type : PID_LOOP_TYPES)
        {
            YAML::Node loop_node = pid_node[PID_LOOP_TYPES_NAMES.at(loop_type)];
            for (const auto &axis : AXES)
            {
                YAML::Node axis_node = loop_node[AXES_NAMES.at(axis)];
                std::shared_ptr<PIDGainsMap> gains = std::make_shared<PIDGainsMap>();

                for (const auto &gain : PID_GAIN_TYPES)
                    (*gains)[gain] = axis_node[PID_GAIN_TYPES_NAMES.at(gain)].as<double>();

                all_pid_gains[loop_type][axis] = gains;
            }
        }

        // Read static power vector
        YAML::Node static_power_global_node = config["static_power_global"];
        static_power_global.setX(static_power_global_node[AXES_NAMES.at(AxesEnum::X)].as<double>());
        static_power_global.setY(static_power_global_node[AXES_NAMES.at(AxesEnum::Y)].as<double>());
        static_power_global.setZ(static_power_global_node[AXES_NAMES.at(AxesEnum::Z)].as<double>());

        // Read power multiplier
        power_multiplier = config["power_multiplier"].as<double>();

        // Read wrench matrix file paths
        wrench_matrix_file_path = config["wrench_matrix_file_path"].as<std::string>();
        wrench_matrix_pinv_file_path = config["wrench_matrix_pinv_file_path"].as<std::string>();

        // Convert wrench matrix file paths to absolute paths
        wrench_matrix_file_path = CONTROLS_PACKAGE_PATH + "/" + wrench_matrix_file_path;
        wrench_matrix_pinv_file_path = CONTROLS_PACKAGE_PATH + "/" + wrench_matrix_pinv_file_path;
    }
    catch (const std::exception &e)
    {
        ROS_ERROR("Exception: %s", e.what());
        ROS_ASSERT_MSG(false, "Could not read robot config file. Make sure it is in the correct format. '%s'",
                       file_path.c_str());
    }
}

void ControlsUtils::update_robot_pid_gains(std::string file_path, const LoopsAxesPIDGainsMap &all_pid_gains)
{
    try
    {
        YAML::Node config = YAML::LoadFile(file_path);

        YAML::Node pid = config["pid"];

        for (const auto &loop_type : PID_LOOP_TYPES)
        {
            YAML::Node loop = pid[PID_LOOP_TYPES_NAMES.at(loop_type)];
            for (const auto &axis : AXES)
            {
                YAML::Node axis_node = loop[AXES_NAMES.at(axis)];
                for (const auto &gain : PID_GAIN_TYPES)
                    axis_node[PID_GAIN_TYPES_NAMES.at(gain)] = all_pid_gains.at(loop_type).at(axis)->at(gain);
            }
        }

        std::ofstream fout(file_path);

        if (!fout.is_open())
            throw;

        fout.exceptions(std::ofstream::failbit | std::ofstream::badbit);

        fout << config;

        if (fout.fail())
            throw;

        fout.close();
    }
    catch (const std::exception &e)
    {
        ROS_ERROR("Exception: %s", e.what());
        ROS_ASSERT_MSG(false, "Could not update pid gains in robot config file. Make sure it is in the correct format. '%s'",
                       file_path.c_str());
    }
}

void ControlsUtils::update_robot_static_power_global(std::string file_path, const tf2::Vector3 &static_power_global)
{
    try
    {
        YAML::Node config = YAML::LoadFile(file_path);

        YAML::Node static_power_global_node = config["static_power_global"];
        static_power_global_node[AXES_NAMES.at(AxesEnum::X)] = static_power_global.getX();
        static_power_global_node[AXES_NAMES.at(AxesEnum::Y)] = static_power_global.getY();
        static_power_global_node[AXES_NAMES.at(AxesEnum::Z)] = static_power_global.getZ();

        std::ofstream fout(file_path);

        if (!fout.is_open())
            throw;

        fout.exceptions(std::ofstream::failbit | std::ofstream::badbit);

        fout << config;

        if (fout.fail())
            throw;

        fout.close();
    }
    catch (const std::exception &e)
    {
        ROS_ERROR("Exception: %s", e.what());
        ROS_ASSERT_MSG(false, "Could not update static power in robot config file. Make sure it is in the correct format. '%s'",
                       file_path.c_str());
    }
}