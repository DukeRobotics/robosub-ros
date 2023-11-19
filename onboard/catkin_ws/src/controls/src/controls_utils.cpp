#include <iostream>
#include <fstream>
#include <sstream>
#include <memory>
#include <unordered_map>
#include <string>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <custom_msgs/ControlTypes.h>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

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

    for (const auto &pair : new_control_types)
        if (!value_in_control_types_enum(pair.second))
            return false;

    for (const auto &pair : new_control_types)
        map[pair.first] = static_cast<ControlTypesEnum>(pair.second);

    return true;
}

void ControlsUtils::map_to_control_types(const std::unordered_map<AxisEnum, ControlTypesEnum> &map,
                                         custom_msgs::ControlTypes &control_types)
{
    control_types.x = map.at(AxisEnum::X);
    control_types.y = map.at(AxisEnum::Y);
    control_types.z = map.at(AxisEnum::Z);
    control_types.roll = map.at(AxisEnum::ROLL);
    control_types.pitch = map.at(AxisEnum::PITCH);
    control_types.yaw = map.at(AxisEnum::YAW);
}

void ControlsUtils::read_matrix_from_csv(std::string file_path, Eigen::MatrixXd &matrix)
{
    // Open the CSV file
    std::ifstream file(file_path);

    ROS_ASSERT_MSG(file.is_open(), "Could not open file. %s", file_path.c_str());

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
                                      LoopsAxesPIDGainsMap &robot_config,
                                      std::string &wrench_matrix_file_path,
                                      std::string &wrench_matrix_pinv_file_path)
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
                std::shared_ptr<PIDGainsMap> gains = std::make_shared<PIDGainsMap>();

                for (const auto &gain : PID_GAIN_TYPES)
                    (*gains)[gain] = axis_node[PID_GAIN_TYPES_NAMES.at(gain)].as<double>();

                robot_config[loop_type][axis] = gains;
            }
        }

        // Read wrench matrix file paths
        wrench_matrix_file_path = config["wrench_matrix_file_path"].as<std::string>();
        wrench_matrix_pinv_file_path = config["wrench_matrix_pinv_file_path"].as<std::string>();
    }
    catch (const std::exception &e)
    {
        ROS_ERROR("Could not read robot config file. Make sure it is in the correct format. %s", file_path.c_str());
        throw;
    }
}