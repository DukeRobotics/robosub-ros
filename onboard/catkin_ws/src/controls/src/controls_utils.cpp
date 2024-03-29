#include "controls_utils.h"

#include <custom_msgs/ControlTypes.h>
#include <custom_msgs/PIDGain.h>
#include <custom_msgs/PIDGains.h>
#include <custom_msgs/ThrusterAllocs.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <unordered_map>

#include "controls_types.h"

std::mutex ControlsUtils::robot_config_mutex;

template <typename T, typename S>
bool ControlsUtils::value_in_array(const T value, const S *array, const size_t &array_size) {
    return std::any_of(array, array + array_size, [value](const S &element) { return value == element; });
}

bool ControlsUtils::value_in_control_types_enum(uint8_t value) {
    return value_in_array<uint8_t, ControlTypesEnum>(value, CONTROL_TYPES, CONTROL_TYPES_COUNT);
}

bool ControlsUtils::value_in_axes_enum(uint8_t value) {
    return value_in_array<uint8_t, AxesEnum>(value, AXES, AXES_COUNT);
}

bool ControlsUtils::value_in_pid_loop_types_enum(uint8_t value) {
    return value_in_array<uint8_t, PIDLoopTypesEnum>(value, PID_LOOP_TYPES, PID_LOOP_TYPES_COUNT);
}

bool ControlsUtils::value_in_pid_gain_types_enum(uint8_t value) {
    return value_in_array<uint8_t, PIDGainTypesEnum>(value, PID_GAIN_TYPES, PID_GAIN_TYPES_COUNT);
}

bool ControlsUtils::value_in_pid_derivative_types_enum(uint8_t value) {
    return value_in_array<uint8_t, PIDDerivativeTypesEnum>(value, PID_DERIVATIVE_TYPES, PID_DERIVATIVE_TYPES_COUNT);
}

bool ControlsUtils::quaternion_valid(const geometry_msgs::Quaternion &quaternion) {
    tf2::Quaternion q;
    tf2::fromMsg(quaternion, q);
    return std::abs(q.length() - 1.0) < 1e-6;
}

bool ControlsUtils::pid_gain_valid(const custom_msgs::PIDGain &pid_gain) {
    return value_in_pid_loop_types_enum(pid_gain.loop) && value_in_axes_enum(pid_gain.axis) &&
           value_in_pid_gain_types_enum(pid_gain.gain);
}

bool ControlsUtils::pid_gains_valid(const std::vector<custom_msgs::PIDGain> &pid_gains) {
    return std::all_of(pid_gains.begin(), pid_gains.end(), pid_gain_valid);
}

bool ControlsUtils::pid_gains_map_valid(const PIDGainsMap &pid_gains_map) {
    return std::all_of(PID_GAIN_TYPES, PID_GAIN_TYPES + PID_GAIN_TYPES_COUNT,
                       [&pid_gains_map](const PIDGainTypesEnum &gain) { return pid_gains_map.count(gain); });
}

void ControlsUtils::quaternion_msg_to_euler(const geometry_msgs::Quaternion &quaternion, double &roll, double &pitch,
                                            double &yaw) {
    // Get roll, pitch, yaw from quaternion
    // The order of rotation is roll, pitch, yaw
    // Roll, pitch, yaw are in radians with range [-pi, pi]
    tf2::Quaternion q;
    tf2::fromMsg(quaternion, q);
    tf2::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
}

void ControlsUtils::pose_to_twist(const geometry_msgs::Pose &pose, geometry_msgs::Twist &twist) {
    twist.linear.x = pose.position.x;
    twist.linear.y = pose.position.y;
    twist.linear.z = pose.position.z;
    quaternion_msg_to_euler(pose.orientation, twist.angular.x, twist.angular.y, twist.angular.z);
}

void ControlsUtils::twist_to_map(const geometry_msgs::Twist &twist, AxesMap<double> &map) {
    map[AxesEnum::X] = twist.linear.x;
    map[AxesEnum::Y] = twist.linear.y;
    map[AxesEnum::Z] = twist.linear.z;
    map[AxesEnum::ROLL] = twist.angular.x;
    map[AxesEnum::PITCH] = twist.angular.y;
    map[AxesEnum::YAW] = twist.angular.z;
}

void ControlsUtils::map_to_twist(const AxesMap<double> &map, geometry_msgs::Twist &twist) {
    twist.linear.x = map.at(AxesEnum::X);
    twist.linear.y = map.at(AxesEnum::Y);
    twist.linear.z = map.at(AxesEnum::Z);
    twist.angular.x = map.at(AxesEnum::ROLL);
    twist.angular.y = map.at(AxesEnum::PITCH);
    twist.angular.z = map.at(AxesEnum::YAW);
}

void ControlsUtils::eigen_vector_to_thruster_allocs_msg(const Eigen::VectorXd &vector,
                                                        custom_msgs::ThrusterAllocs &thruster_allocs) {
    thruster_allocs.allocs.clear();
    for (int i = 0; i < vector.rows(); ++i) thruster_allocs.allocs.push_back(vector(i));
}

void ControlsUtils::eigen_vector_to_twist(const Eigen::VectorXd &vector, geometry_msgs::Twist &twist) {
    twist.linear.x = vector(0);
    twist.linear.y = vector(1);
    twist.linear.z = vector(2);
    twist.angular.x = vector(3);
    twist.angular.y = vector(4);
    twist.angular.z = vector(5);
}

void ControlsUtils::eigen_vector_to_map(const Eigen::VectorXd &vector, AxesMap<double> &map) {
    map[AxesEnum::X] = vector(0);
    map[AxesEnum::Y] = vector(1);
    map[AxesEnum::Z] = vector(2);
    map[AxesEnum::ROLL] = vector(3);
    map[AxesEnum::PITCH] = vector(4);
    map[AxesEnum::YAW] = vector(5);
}

bool ControlsUtils::control_types_to_map(const custom_msgs::ControlTypes &control_types,
                                         AxesMap<ControlTypesEnum> &map) {
    AxesMap<uint8_t> new_control_types;
    new_control_types[AxesEnum::X] = control_types.x;
    new_control_types[AxesEnum::Y] = control_types.y;
    new_control_types[AxesEnum::Z] = control_types.z;
    new_control_types[AxesEnum::ROLL] = control_types.roll;
    new_control_types[AxesEnum::PITCH] = control_types.pitch;
    new_control_types[AxesEnum::YAW] = control_types.yaw;

    for (const auto &pair : new_control_types)
        if (!value_in_control_types_enum(pair.second)) return false;

    for (const auto &pair : new_control_types) map[pair.first] = static_cast<ControlTypesEnum>(pair.second);

    return true;
}

void ControlsUtils::map_to_control_types(const AxesMap<ControlTypesEnum> &map,
                                         custom_msgs::ControlTypes &control_types) {
    control_types.x = map.at(AxesEnum::X);
    control_types.y = map.at(AxesEnum::Y);
    control_types.z = map.at(AxesEnum::Z);
    control_types.roll = map.at(AxesEnum::ROLL);
    control_types.pitch = map.at(AxesEnum::PITCH);
    control_types.yaw = map.at(AxesEnum::YAW);
}

void ControlsUtils::tf_linear_vector_to_map(const tf2::Vector3 &vector, AxesMap<double> &map) {
    map[AxesEnum::X] = vector.getX();
    map[AxesEnum::Y] = vector.getY();
    map[AxesEnum::Z] = vector.getZ();
    map[AxesEnum::ROLL] = 0.0;
    map[AxesEnum::PITCH] = 0.0;
    map[AxesEnum::YAW] = 0.0;
}

void ControlsUtils::pid_loops_axes_gains_map_to_msg(const LoopsMap<AxesMap<PIDGainsMap>> &loops_axes_pid_gains,
                                                    custom_msgs::PIDGains &pid_gains_msg) {
    pid_gains_msg.pid_gains.clear();

    for (const PIDLoopTypesEnum &loop : PID_LOOP_TYPES)
        for (const AxesEnum &axis : AXES)
            for (const PIDGainTypesEnum &gain : PID_GAIN_TYPES) {
                custom_msgs::PIDGain pid_gain;
                pid_gain.loop = loop;
                pid_gain.axis = axis;
                pid_gain.gain = gain;
                pid_gain.value = loops_axes_pid_gains.at(loop).at(axis).at(gain);
                pid_gains_msg.pid_gains.push_back(pid_gain);
            }
}

void ControlsUtils::pid_terms_struct_to_msg(const PIDTerms &terms, custom_msgs::PIDTerms &terms_msg) {
    terms_msg.proportional = terms.proportional;
    terms_msg.integral = terms.integral;
    terms_msg.derivative = terms.derivative;
    terms_msg.feedforward = terms.feedforward;
}

void ControlsUtils::pid_info_struct_to_msg(const PIDInfo &pid_info, custom_msgs::PIDInfo &pid_info_msg) {
    pid_info_msg.terms = custom_msgs::PIDTerms();
    pid_terms_struct_to_msg(pid_info.terms, pid_info_msg.terms);

    pid_info_msg.filtered_error = pid_info.filtered_error;
    pid_info_msg.integral = pid_info.integral;
    pid_info_msg.filtered_derivative = pid_info.filtered_derivative;

    pid_info_msg.calculated_derivative = pid_info.calculated_derivative;
    pid_info_msg.provided_derivative = pid_info.provided_derivative;
    pid_info_msg.derivative_type.type = pid_info.derivative_type;
}

void ControlsUtils::pid_axes_map_info_struct_to_msg(const AxesMap<PIDInfo> &pid_axes_map_info_struct,
                                                    custom_msgs::PIDAxesInfo &pid_axes_info_msg) {
    pid_info_struct_to_msg(pid_axes_map_info_struct.at(AxesEnum::X), pid_axes_info_msg.x);
    pid_info_struct_to_msg(pid_axes_map_info_struct.at(AxesEnum::Y), pid_axes_info_msg.y);
    pid_info_struct_to_msg(pid_axes_map_info_struct.at(AxesEnum::Z), pid_axes_info_msg.z);
    pid_info_struct_to_msg(pid_axes_map_info_struct.at(AxesEnum::ROLL), pid_axes_info_msg.roll);
    pid_info_struct_to_msg(pid_axes_map_info_struct.at(AxesEnum::PITCH), pid_axes_info_msg.pitch);
    pid_info_struct_to_msg(pid_axes_map_info_struct.at(AxesEnum::YAW), pid_axes_info_msg.yaw);
}

template <typename T>
void ControlsUtils::populate_axes_map(const T &value, AxesMap<T> &map) {
    for (const AxesEnum &axis : AXES) map[axis] = value;
}

template void ControlsUtils::populate_axes_map<double>(const double &value, AxesMap<double> &map);
template void ControlsUtils::populate_axes_map<ControlTypesEnum>(const ControlTypesEnum &value,
                                                                 AxesMap<ControlTypesEnum> &map);

void ControlsUtils::scale_axes_map(const double &scale_factor, AxesMap<double> &map) {
    for (const AxesEnum &axis : AXES) map[axis] *= scale_factor;
}

void ControlsUtils::read_matrix_from_csv(const std::string &file_path, Eigen::MatrixXd &matrix) {
    // Open the CSV file
    std::ifstream file(file_path);

    ROS_ASSERT_MSG(file.is_open(), "Could not open CSV file. '%s'", file_path.c_str());

    // Temporarily store matrix as vector of vectors
    std::vector<std::vector<double>> data;

    // One row of the file
    std::string line;

    // Number of columns in the matrix
    int cols = -1;

    // Read the file row by row
    while (std::getline(file, line)) {
        // Stream of the current row
        // Allows reading the row token by token
        // Tokens differentiated by their type (double or char)
        // Whitespaces are ignored
        std::istringstream iss(line);

        // Vector of values in this row
        std::vector<double> row;

        // Next numeric value
        double value;

        // Comma following the value
        char comma;

        // In each iteration of the loop, the next numeric token is read, along with the comma that follows it
        // The loop condition will be true if the next token is numeric and was stored in `value`
        // The loop condition will be false if the next token is not numeric or if the end of the line is reached
        while (iss >> value) {
            // Add the value to the list of values in this row
            row.push_back(value);

            // Read the comma
            // If the end of the line is reached, `comma` will not be updated
            iss >> comma;

            // Ensure every value has comma succeeding it (unless it is the last value in the row)
            ROS_ASSERT_MSG((iss.good() && comma == ',') || iss.eof(), "File is not in valid CSV format. '%s'",
                           file_path.c_str());
        }

        // Ensure the file contains only numeric values
        // If the file contains non-numeric values, the loop will have exited before the line was read completely
        ROS_ASSERT_MSG(iss.eof(), "CSV file contains non-numeric values. '%s'", file_path.c_str());

        // Ensure all rows have the same number of columns
        if (cols == -1)
            cols = row.size();
        else
            ROS_ASSERT_MSG(row.size() == cols, "CSV file must have same number of columns in all rows. '%s'",
                           file_path.c_str());

        // Add the row to the temporary matrix
        data.push_back(row);
    }

    // Close the file
    file.close();

    // Determine the matrix size
    int rows = data.size();

    // Ensure the matrix has at least one row and one column
    ROS_ASSERT_MSG(cols >= 1, "CSV file must have at least one column. '%s'", file_path.c_str());
    ROS_ASSERT_MSG(rows >= 1, "CSV file must have at least one row. '%s'", file_path.c_str());

    // Initialize the Eigen matrix
    matrix.resize(rows, cols);

    // Copy the data from the vector of vectors to the Eigen matrix
    for (int i = 0; i < rows; ++i)
        for (int j = 0; j < cols; ++j) matrix(i, j) = data[i][j];
}

void ControlsUtils::read_robot_config(const bool &cascaded_pid,
                                      LoopsMap<AxesMap<double>> &loops_axes_control_effort_mins,
                                      LoopsMap<AxesMap<double>> &loops_axes_control_effort_maxes,
                                      LoopsMap<AxesMap<PIDDerivativeTypesEnum>> &loops_axes_derivative_types,
                                      LoopsMap<AxesMap<double>> &loops_axes_error_ramp_rates,
                                      LoopsMap<AxesMap<PIDGainsMap>> &loops_axes_pid_gains,
                                      AxesMap<double> &desired_power_min, AxesMap<double> &desired_power_max,
                                      tf2::Vector3 &static_power_global, double &power_scale_factor,
                                      std::string &wrench_matrix_file_path, std::string &wrench_matrix_pinv_file_path) {
    try {
        // Lock the mutex to prevent other threads from accessing the robot config file while it is being read
        std::lock_guard<std::mutex> guard(robot_config_mutex);

        // Read the robot config file into a YAML node
        YAML::Node config = YAML::LoadFile(ROBOT_CONFIG_FILE_PATH);

        // Read PID configurations for each loop and axis
        YAML::Node pid_node = config["pid"];
        for (const auto &loop : PID_LOOP_TYPES) {
            // Read gains for cascaded position PID loop if cascaded PID is enabled
            std::string loop_name = (loop == PIDLoopTypesEnum::POSITION && cascaded_pid)
                                        ? CASCADED_POSITION_PID_NAME
                                        : PID_LOOP_TYPES_NAMES.at(loop);
            YAML::Node loop_node = pid_node[loop_name];
            for (const auto &axis : AXES) {
                YAML::Node axis_node = loop_node[AXES_NAMES.at(axis)];

                // Read control effort limits, derivative types, and error ramp rates
                loops_axes_control_effort_mins[loop][axis] = axis_node["control_effort"]["min"].as<double>();
                loops_axes_control_effort_maxes[loop][axis] = axis_node["control_effort"]["max"].as<double>();
                loops_axes_derivative_types[loop][axis] =
                    static_cast<PIDDerivativeTypesEnum>(axis_node["derivative_type"].as<int>());
                loops_axes_error_ramp_rates[loop][axis] = axis_node["error_ramp_rate"].as<double>();

                // Read PID gains
                PIDGainsMap gains = PIDGainsMap();
                for (const auto &gain : PID_GAIN_TYPES)
                    gains[gain] = axis_node[PID_GAIN_TYPES_NAMES.at(gain)].as<double>();

                loops_axes_pid_gains[loop][axis] = gains;
            }
        }

        // Read desired power limits
        YAML::Node desired_power_limits_node = config["desired_power_limits"];
        for (const auto &axis : AXES) {
            desired_power_min[axis] = desired_power_limits_node[AXES_NAMES.at(axis)]["min"].as<double>();
            desired_power_max[axis] = desired_power_limits_node[AXES_NAMES.at(axis)]["max"].as<double>();
        }

        // Read static power vector
        YAML::Node static_power_global_node = config["static_power_global"];
        static_power_global.setX(static_power_global_node[AXES_NAMES.at(AxesEnum::X)].as<double>());
        static_power_global.setY(static_power_global_node[AXES_NAMES.at(AxesEnum::Y)].as<double>());
        static_power_global.setZ(static_power_global_node[AXES_NAMES.at(AxesEnum::Z)].as<double>());

        // Read power multiplier
        power_scale_factor = config["power_scale_factor"].as<double>();

        // Read wrench matrix file paths
        wrench_matrix_file_path = config["wrench_matrix_file_path"].as<std::string>();
        wrench_matrix_pinv_file_path = config["wrench_matrix_pinv_file_path"].as<std::string>();

        // Convert wrench matrix file paths to absolute paths
        wrench_matrix_file_path = CONTROLS_PACKAGE_PATH + "/" + wrench_matrix_file_path;
        wrench_matrix_pinv_file_path = CONTROLS_PACKAGE_PATH + "/" + wrench_matrix_pinv_file_path;
    } catch (const std::exception &e) {
        ROS_ERROR("Exception: %s", e.what());
        ROS_ASSERT_MSG(false, "Could not read robot config file. Make sure it is in the correct format. '%s'",
                       ROBOT_CONFIG_FILE_PATH.c_str());
    }
}

void ControlsUtils::update_robot_config(std::function<void(YAML::Node &)> update_function, std::string update_name) {
    try {
        // Lock the mutex to prevent other threads from accessing the robot config file while it is being updated
        std::lock_guard<std::mutex> guard(robot_config_mutex);

        // Read the robot config file into a YAML node and update it with the provided function
        YAML::Node config = YAML::LoadFile(ROBOT_CONFIG_FILE_PATH);
        update_function(config);

        // Open the robot config file for writing
        std::ofstream fout(ROBOT_CONFIG_FILE_PATH);
        if (!fout.is_open()) throw;

        // Set exceptions to be thrown on failure
        fout.exceptions(std::ofstream::failbit | std::ofstream::badbit);

        // Write the updated YAML node to the file
        fout << config;
        if (fout.fail()) throw;

        fout.close();
    } catch (const std::exception &e) {
        ROS_ERROR("Exception: %s", e.what());
        ROS_ASSERT_MSG(false, "Could not update %s in robot config file. Make sure it is in the correct format. '%s'",
                       update_name.c_str(), ROBOT_CONFIG_FILE_PATH.c_str());
    }
}

void ControlsUtils::update_robot_config_pid_gains(const LoopsMap<AxesMap<PIDGainsMap>> &loops_axes_pid_gains,
                                                  const bool &cascaded_pid) {
    std::function<void(YAML::Node &)> update_function = [&loops_axes_pid_gains, &cascaded_pid](YAML::Node &config) {
        // Update PID gains for each loop and axis
        YAML::Node pid_node = config["pid"];
        for (const auto &loop : PID_LOOP_TYPES) {
            // Update gains for cascaded position PID loop if cascaded PID is enabled
            std::string loop_name = (loop == PIDLoopTypesEnum::POSITION && cascaded_pid)
                                        ? CASCADED_POSITION_PID_NAME
                                        : PID_LOOP_TYPES_NAMES.at(loop);
            YAML::Node loop_node = pid_node[loop_name];
            for (const auto &axis : AXES) {
                YAML::Node axis_node = loop_node[AXES_NAMES.at(axis)];
                for (const auto &gain : PID_GAIN_TYPES)
                    axis_node[PID_GAIN_TYPES_NAMES.at(gain)] = loops_axes_pid_gains.at(loop).at(axis).at(gain);
            }
        }
    };

    update_robot_config(update_function, "PID gains");
}

void ControlsUtils::update_robot_config_static_power_global(const tf2::Vector3 &static_power_global) {
    std::function<void(YAML::Node &)> update_function = [&static_power_global](YAML::Node &config) {
        YAML::Node static_power_global_node = config["static_power_global"];
        static_power_global_node[AXES_NAMES.at(AxesEnum::X)] = static_power_global.getX();
        static_power_global_node[AXES_NAMES.at(AxesEnum::Y)] = static_power_global.getY();
        static_power_global_node[AXES_NAMES.at(AxesEnum::Z)] = static_power_global.getZ();
    };

    update_robot_config(update_function, "static power global");
}

void ControlsUtils::update_robot_config_power_scale_factor(double &power_scale_factor) {
    std::function<void(YAML::Node &)> update_function = [&power_scale_factor](YAML::Node &config) {
        config["power_scale_factor"] = power_scale_factor;
    };

    update_robot_config(update_function, "power scale factor");
}

double ControlsUtils::clip(const double value, const double min, const double max) {
    return std::max(min, std::min(value, max));
}