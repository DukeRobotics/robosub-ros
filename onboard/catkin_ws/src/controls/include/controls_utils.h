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
#include <custom_msgs/PIDAxesInfo.h>
#include <custom_msgs/PIDDerivativeType.h>
#include <custom_msgs/PIDGain.h>
#include <custom_msgs/PIDGains.h>
#include <custom_msgs/PIDInfo.h>
#include <custom_msgs/PIDTerms.h>
#include <custom_msgs/ThrusterAllocs.h>
#include <Eigen/Dense>

// Control types: DESIRED_POSE, DESIRED_TWIST, DESIRED_POWER
enum ControlTypesEnum : uint8_t
{
    DESIRED_POSE = custom_msgs::ControlTypes::DESIRED_POSE,
    DESIRED_TWIST = custom_msgs::ControlTypes::DESIRED_TWIST,
    DESIRED_POWER = custom_msgs::ControlTypes::DESIRED_POWER
};

// Axes: X, Y, Z, ROLL, PITCH, YAW
enum AxesEnum : uint8_t
{
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
const AxesEnum AXES[AXES_COUNT] = {AxesEnum::X, AxesEnum::Y, AxesEnum::Z,
                                   AxesEnum::ROLL, AxesEnum::PITCH, AxesEnum::YAW};

// Axes string names
const std::unordered_map<AxesEnum, std::string> AXES_NAMES = {
    {AxesEnum::X, "x"},
    {AxesEnum::Y, "y"},
    {AxesEnum::Z, "z"},
    {AxesEnum::ROLL, "roll"},
    {AxesEnum::PITCH, "pitch"},
    {AxesEnum::YAW, "yaw"}};

// PID loop types: POSITION, VELOCITY
enum PIDLoopTypesEnum : uint8_t
{
    POSITION = custom_msgs::PIDGain::LOOP_POSITION,
    VELOCITY = custom_msgs::PIDGain::LOOP_VELOCITY
};

// Number of PID loop types
const int PID_LOOP_TYPES_COUNT = 2;

// PID loop types array
const PIDLoopTypesEnum PID_LOOP_TYPES[PID_LOOP_TYPES_COUNT] = {PIDLoopTypesEnum::POSITION, PIDLoopTypesEnum::VELOCITY};

// PID loop types string names
const std::unordered_map<PIDLoopTypesEnum, std::string> PID_LOOP_TYPES_NAMES = {
    {PIDLoopTypesEnum::POSITION, "position"},
    {PIDLoopTypesEnum::VELOCITY, "velocity"}};

// PID gain types: KP, KI, KD, FF
enum PIDGainTypesEnum : uint8_t
{
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
const std::unordered_map<PIDGainTypesEnum, std::string> PID_GAIN_TYPES_NAMES = {
    {PIDGainTypesEnum::KP, "Kp"},
    {PIDGainTypesEnum::KI, "Ki"},
    {PIDGainTypesEnum::KD, "Kd"},
    {PIDGainTypesEnum::FF, "Ff"}};

// Path to controls package
const std::string CONTROLS_PACKAGE_PATH = ros::package::getPath("controls");

// Path to robot config file
const std::string ROBOT_CONFIG_FILE_PATH = CONTROLS_PACKAGE_PATH + "/config/" + std::string(std::getenv("ROBOT_NAME")) + ".yaml";

// Map of PID gains
typedef std::unordered_map<PIDGainTypesEnum, double> PIDGainsMap;

// Map of axes to template type
template <typename T>
using AxesMap = std::unordered_map<AxesEnum, T>;

// Map of PID loop types to axes to template type
template <typename T>
using LoopsMap = std::unordered_map<PIDLoopTypesEnum, T>;

// PID Derivative Types: CALCULATED, PROVIDED
enum PIDDerivativeTypesEnum : uint8_t
{
    CALCULATED = custom_msgs::PIDDerivativeType::CALCULATED,
    PROVIDED = custom_msgs::PIDDerivativeType::PROVIDED
};

// PID terms that are summed to get control effort
struct PIDTerms
{
    double proportional;
    double integral;
    double derivative;
    double feedforward;
};

// Values computed by PID loop
struct PIDInfo
{
    PIDTerms terms;

    double filtered_error;
    double integral;
    double filtered_derivative;

    double calculated_derivative;
    double provided_derivative;
    PIDDerivativeTypesEnum derivative_type;
};

namespace ControlsUtils
{
    // Mutex for robot config file.
    // Allows only one thread to read or write to robot config file at a time.
    extern std::mutex robot_config_mutex;

    // *****************************************************************************************************************
    // Functions to check if value is in enum.

    /**
     * @brief Check if value is in `ControlTypesEnum`.
     *
     * @param value Value to check.
     * @return True if value is in `ControlTypesEnum`, false otherwise.
     */
    bool value_in_control_types_enum(uint8_t value);

    /**
     * @brief Check if value is in `AxesEnum`.
     *
     * @param value Value to check.
     * @return True if value is in `AxesEnum`, false otherwise.
     */
    bool value_in_axes_enum(uint8_t value);

    /**
     * @brief Check if value is in `PIDLoopTypesEnum`.
     *
     * @param value Value to check.
     * @return True if value is in `PIDLoopTypesEnum`, false otherwise.
     */
    bool value_in_pid_loop_types_enum(uint8_t value);

    /**
     * @brief Check if value is in `PIDGainTypesEnum`.
     *
     * @param value Value to check.
     * @return True if value is in `PIDGainTypesEnum`, false otherwise.
     */
    bool value_in_pid_gain_types_enum(uint8_t value);

    /**
     * @brief Check if value is in `PIDDerivativeTypesEnum`.
     *
     * @param value Value to check.
     * @return True if value is in `PIDDerivativeTypesEnum`, false otherwise.
     */
    bool value_in_pid_derivative_types_enum(uint8_t value);

    // *****************************************************************************************************************
    // Functions to check if message is valid.

    /**
     * @brief Check if quaternion has length 1 (within tolerance of 1e-6).
     *
     * @param quaternion Quaternion to check.
     * @return True if quaternion has length 1, false otherwise.
     */
    bool quaternion_valid(const geometry_msgs::Quaternion &quaternion);

    /**
     * @brief Check if twist is in range.
     *
     * @param twist Twist to check.
     * @param min Minimum value.
     * @param max Maximum value.
     * @return True if all values in twist are in range [`min`, `max`], false otherwise.
     */
    bool twist_in_range(const geometry_msgs::Twist &twist, double min, double max);

    /**
     * @brief Check if pid gain message has valid loop, axis, and gain types.
     *
     * @param pid_gain PID gain message to check.
     * @return True if pid gain message has valid loop, axis, and gain types, false otherwise.
     */
    bool pid_gain_valid(const custom_msgs::PIDGain &pid_gain);

    /**
     * @brief Check if all pid gains in given pid gains message have valid loop, axis, and gain types.
     *
     * @param pid_gains PID gains message to check.
     * @return True if all pid gains messages in `pid_gains` have valid loop, axis, and gain types, false otherwise.
     */
    bool pid_gains_valid(const std::vector<custom_msgs::PIDGain> &pid_gains);

    /**
     * @brief Check if pid gains map has values for all four gain types.
     *
     * @param pid_gains_map PID gains map to check.
     * @return True if pid gains map has values for all four gain types, false otherwise.
     */
    bool pid_gains_map_valid(const PIDGainsMap &pid_gains_map);

    // *****************************************************************************************************************
    // Functions to convert between types.

    /**
     * @brief Convert pose to twist. Linear vector is identical to position. Angular vector is orientation converted to
     *  euler angles.
     *
     * @param pose Pose to convert.
     * @param twist Twist to populate.
     */
    void pose_to_twist(const geometry_msgs::Pose &pose, geometry_msgs::Twist &twist);

    /**
     * @brief Convert twist to axes map.
     *
     * @param twist Twist to convert.
     * @param map Map to populate.
     */
    void twist_to_map(const geometry_msgs::Twist &twist, AxesMap<double> &map);

    /**
     * @brief Convert axes map to twist.
     *
     * @param map Map to convert.
     * @param twist Twist to populate.
     */
    void map_to_twist(const AxesMap<double> &map, geometry_msgs::Twist &twist);

    /**
     * @brief Convert Eigen vector to thruster allocs message. `thruster_allocs.allocs` is first cleared. Then,
     *  `vector`'s entries are mapped from top to bottom to `thruster_allocs.allocs` in left to right order. The topmost
     *  entry in `vector` is mapped to the leftmost entry in `thruster_allocs.allocs`; the bottommost entry in `vector`
     *  is mapped to the rightmost entry in `thruster_allocs.allocs`.
     *
     * @param vector Eigen vector to convert.
     * @param thruster_allocs Thruster allocs message to populate.
     */
    void eigen_vector_to_thruster_allocs(const Eigen::VectorXd &vector, custom_msgs::ThrusterAllocs &thruster_allocs);

    /**
     * @brief Convert Eigen vector to twist. `vector`'s entries are mapped from top to bottom to `twist` in the
     *  following order: linear.x, linear.y, linear.z, angular.x, angular.y, angular.z.
     *
     * @param vector Eigen vector to convert.
     * @param twist Twist to populate.
     */
    void eigen_vector_to_twist(const Eigen::VectorXd &vector, geometry_msgs::Twist &twist);

    /**
     * @brief Convert Eigen vector to axes map. `vector`'s entries are mapped from top to bottom to `map` in the
     *  following order: X, Y, Z, ROLL, PITCH, YAW.
     *
     * @param vector Eigen vector to convert.
     * @param map Axes map to populate.
     */
    void eigen_vector_to_map(const Eigen::VectorXd &vector, AxesMap<double> &map);

    /**
     * @brief Convert control types message to axes map. Checks if message is valid before modifying `map`.
     *
     * @param control_types Control types message to convert.
     * @param map Axes map to populate.
     * @return True if message is valid and conversion was performed, false otherwise.
     */
    bool control_types_to_map(const custom_msgs::ControlTypes &control_types, AxesMap<ControlTypesEnum> &map);

    /**
     * @brief Convert axes map to control types message.
     *
     * @param map Axes map to convert.
     * @param control_types Control types message to populate.
     */
    void map_to_control_types(const AxesMap<ControlTypesEnum> &map, custom_msgs::ControlTypes &control_types);

    /**
     * @brief Convert tf Vector3 to axes map.
     *
     * @details `vector`'s X, Y, and Z entries are mapped to `map`'s X, Y, and Z entries; `map`'s ROLL, PITCH, and YAW
     *  entries are set to 0.
     *
     * @param vector Vector3 to convert.
     * @param map Axes map to populate.
     */
    void tf_linear_vector_to_map(const tf2::Vector3 &vector, AxesMap<double> &map);

    /**
     * @brief Convert map of loops to axes to pid gains to pid gains message.
     *
     * @param loops_axes_pid_gains Map of loops to axes to pid gains to convert.
     * @param pid_gains_msg PID gains message to populate.
     */
    void pid_loops_axes_gains_map_to_msg(const LoopsMap<AxesMap<PIDGainsMap>> &loops_axes_pid_gains,
                                         custom_msgs::PIDGains &pid_gains_msg);

    /**
     * @brief Convert pid terms struct to pid terms message.
     *
     * @param terms PID terms struct to convert.
     * @param terms_msg PID terms message to populate.
     */
    void pid_terms_struct_to_msg(const PIDTerms &terms, custom_msgs::PIDTerms &terms_msg);

    /**
     * @brief Convert pid info struct to pid info message.
     *
     * @param pid_info PID info struct to convert.
     * @param pid_info_msg PID info message to populate.
     */
    void pid_info_struct_to_msg(const PIDInfo &pid_info, custom_msgs::PIDInfo &pid_info_msg);

    /**
     * @brief Convert axes map with pid info structs to pid axes info message.
     *
     * @param pid_axes_map_info_struct Axes map with PID info structs to convert.
     * @param pid_axes_info_msg PID info message to populate.
     */
    void pid_axes_map_info_struct_to_msg(const AxesMap<PIDInfo> &pid_axes_map_info_struct,
                                         custom_msgs::PIDAxesInfo &pid_axes_info_msg);

    // *****************************************************************************************************************
    // Functions to update maps.

    /**
     * @brief Set values for all axes in `map` to `value`.
     *
     * @param map Map to populate.
     * @param value Value to set all axes to.
     */
    void populate_axes_map(AxesMap<double> &map, double value);

    // *****************************************************************************************************************
    // Functions to read and write to files.

    /**
     * @brief Read matrix from CSV file.
     *
     * @details Each row of the CSV file is a row of the matrix. Each entry in each row must be separated by a comma.
     *  The file must not contain row or column headers. The matrix is resized to the size of the matrix in the
     *  file.
     *
     * @param file_path Path to csv file.
     * @param matrix Matrix to populate.
     *
     * @throws ros::Exception File cannot be opened
     * @throws ros::Exception File is not in the correct format
     * @throws ros::Exception File does not have the same number of columns in each row
     * @throws ros::Exception File does not have at least one row and one column
     */
    void read_matrix_from_csv(std::string file_path, Eigen::MatrixXd &matrix);

    /**
     * @brief Read robot config file and populate variables.
     *
     * @details The robot config file must contain the following fields: `pid`, `static_power_global`,
     *  `power_scale_factor`, `wrench_matrix_file_path`, `wrench_matrix_pinv_file_path`. The `pid` field must contain
     *  subfields for each PID loop type, each of which must contain subfields for each axis, each of which must contain
     *  subfields for each PID gain type. The `static_power_global` field must contain subfields for each axis. The
     *  `wrench_matrix_file_path` and `wrench_matrix_pinv_file_path` fields must contain the file paths relative to the
     *  controls package.
     *
     * @param loops_axes_control_effort_limits Map of PID loop types to axes to control effort limits to populate.
     * @param loops_axes_derivative_types Map of PID loop types to axes to derivative types to populate.
     * @param loops_axes_error_ramp_rates Map of PID loop types to axes to error ramp rates to populate.
     * @param loops_axes_pid_gains Map of PID loop types to axes to PID gains to populate.
     * @param static_power_global Static power global vector to populate.
     * @param power_scale_factor Power scale factor to populate.
     * @param wrench_matrix_file_path Wrench matrix file path to populate.
     * @param wrench_matrix_pinv_file_path Wrench matrix pseudoinverse file path to populate.
     *
     * @throws ros::Exception File cannot be opened
     * @throws ros::Exception File is not in valid YAML format
     * @throws ros::Exception File does not contain all required fields in correct formats
     */
    void read_robot_config(LoopsMap<AxesMap<double>> &loops_axes_control_effort_limits,
                           LoopsMap<AxesMap<PIDDerivativeTypesEnum>> &loops_axes_derivative_types,
                           LoopsMap<AxesMap<double>> &loops_axes_error_ramp_rates,
                           LoopsMap<AxesMap<PIDGainsMap>> &loops_axes_pid_gains,
                           tf2::Vector3 &static_power_global,
                           double &power_scale_factor,
                           std::string &wrench_matrix_file_path,
                           std::string &wrench_matrix_pinv_file_path);

    /**
     * @brief Update robot config file with given `update_function`.
     *
     * @param update_function Function that updates YAML node containing robot config.
     * @param update_name Name of value(s) being updated (used only for error messages).
     *
     * @throws ros::Exception File cannot be opened
     * @throws ros::Exception File is not in valid YAML format
     * @throws ros::Exception File does not contain all fields required by `update_function` in correct formats
     */
    void update_robot_config(std::function<void(YAML::Node &)> update_function, std::string update_name);

    /**
     * @brief Update PID gains in robot config file to match those in `loops_axes_pid_gains`.
     *
     * @param loops_axes_pid_gains Map of PID loops to axes to gains to set in robot config file.
     */
    void update_robot_config_pid_gains(const LoopsMap<AxesMap<PIDGainsMap>> &loops_axes_pid_gains);

    /**
     * @brief Update static power global vector in robot config file to match `static_power_global`.
     *
     * @param static_power_global Static power global vector to set in robot config file.
     */
    void update_robot_config_static_power_global(const tf2::Vector3 &static_power_global);

    /**
     * @brief Update power scale factor in robot config file to match `power_scale_factor`.
     *
     * @param power_scale_factor Power scale factor to set in robot config file.
     */
    void update_robot_config_power_scale_factor(double &power_scale_factor);

    // *****************************************************************************************************************
    // Other functions.

    /**
     * @brief Clip value between `min` and `max`. If `value` is less than `min`, return `min`. If `value` is greater
     *  than `max`, return `max`. Otherwise, return `value`.
     *
     * @param value The value to clip.
     * @param min The minimum value.
     * @param max The maximum value.
     * @return The clipped value, guaranteed to be between `min` and `max` (inclusive).
     */
    double clip(const double value, const double min, const double max);
};

#endif