#include "thrusters.h"

#include <custom_msgs/PWMAllocs.h>
#include <ros/assert.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include <fstream>
#include <sstream>
#include <vector>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

Thrusters::Thrusters(int argc, char **argv, ros::NodeHandle &nh) {

    std::string device;
    if (!nh.getParam("device", device)) {
        ROS_ERROR("Failed to get parameter 'device'");
        return;
    }

    if (device.find("/dev/") == std::string::npos) {
        ROS_ERROR("Invalid device path: %s", device.c_str());
        return;
    }

    ROS_INFO("Thruster serial port: %s", device.c_str());

    // Open the serial port
    serial_fd = open(device.c_str(), O_WRONLY | O_NOCTTY | O_NONBLOCK); // write only, no controlling terminal, non-blocking
    if (serial_fd == -1) {
        ROS_ERROR("Failed to open serial port %s", device.c_str());
        return;
    }

    // Configure serial port settings
    struct termios tty;
    if (tcgetattr(serial_fd, &tty) != 0) {
        ROS_ERROR("Error %d from tcgetattr", errno);
        close(serial_fd);
        return;
    }

    tty.c_cflag &= ~CBAUD; // Clear baud rate bits
    tty.c_cflag |= B57600; // Set baud to 57600

    if (tcsetattr(serial_fd, TCSANOW, &tty) != 0) {
        ROS_ERROR("Error %d from tcsetattr", errno);
        close(serial_fd);
        return;
    }

    ROS_INFO("Thrusters initialized");

    // Set the default voltage
    voltage = 15.5;

    // Read in corresponding voltage tables for 14.0, 16.0, and 18.0V
    load_lookup_tables();

    // Instantiate a subscriber to the voltage sensor, should read voltages from range in [14.0, 18.0]
    voltage_sub = nh.subscribe("/sensors/voltage", 1, &Thrusters::voltage_callback, this);

    // Subscribe to thruster allocation (desired range in [-1.0, 1.0])
    thruster_allocs_sub = nh.subscribe("/controls/thruster_allocs", 1, &Thrusters::thruster_allocs_callback, this);

    // Publish pwm allocation based on calculated conversions
    // Debug purposes only
    pwm_pub = nh.advertise<custom_msgs::PWMAllocs>("/offboard/pwm", 1);
}

Thrusters::~Thrusters() {
    if (serial_fd != -1) {
        close(serial_fd);
    }
}

// This method reads in the csv files for 14.0, 16.0, 18.0V volt conversions
// Each lookup table stores all thruster allocations from -1.0 to 1.0 in 0.01 increments
// with voltage in 14.0 to 18.0V and the appropriate pwm output
void Thrusters::load_lookup_tables() {
    std::string package_path = ros::package::getPath("offboard_comms");

    read_lookup_table_csv(package_path + "/data/14.csv", v14_lookup_table);
    read_lookup_table_csv(package_path + "/data/16.csv", v16_lookup_table);
    read_lookup_table_csv(package_path + "/data/18.csv", v18_lookup_table);
}

// Read lookup table for voltage, thruster alloc, and pwm alloc; thurster alloc/force is stored with 2 decimal precision
void Thrusters::read_lookup_table_csv(const std::string &filename,
                                      std::array<uint16_t, NUM_LOOKUP_ENTRIES> &lookup_table) {
    std::ifstream file(filename);
    ROS_ASSERT_MSG(file.is_open(), "Error opening file %s", filename.c_str());

    std::string line, col;

    // Read header line (assuming headers exist and need to be skipped)
    std::getline(file, line);

    // Read each line of the file
    while (std::getline(file, line)) {
        std::istringstream ss(line);
        std::vector<std::string> row;

        // Read each column of the line and store each column in the row vector
        while (std::getline(ss, col, ',')) {
            row.push_back(col);
        }

        if (row.size() >= 2) {
            // Assuming PWM is in the first column and force is in the second column
            double force = std::stod(row[0]);
            int pwm = std::stoi(row[1]);

            // If the PWM value is within bounds of uint16_t, cast it to uint16_t
            uint16_t pwm16;
            ROS_ASSERT_MSG(pwm <= static_cast<int>(UINT16_MAX) && pwm >= 0,
                           "Error while reading CSV. PWM value %d is out of bounds for uint16_t.", pwm);
            pwm16 = static_cast<uint16_t>(pwm);

            // Multiply the force value by 100 and add 100
            // This is done to convert the force value to an index for the lookup table
            // It is important to use std::lround() instead of type casting to int directly because a direct typecast
            // can result in the incorrect index being calculated (an error of +/- 1)
            int index = round_to_two_decimals(force);

            // If the index is within bounds of the array, store the pwm value at that index
            ROS_ASSERT_MSG(index >= 0 && index < lookup_table.size(),
                           "Error while reading CSV. Index %d is out of bounds.", index);
            lookup_table[index] = pwm16;
        }
    }
    file.close();
}

// Update the voltage field; used in pwm interpolation
void Thrusters::voltage_callback(const std_msgs::Float64 &msg) {
    // Clamp voltage to be within bounds
    voltage = std::max(VOLTAGE_LOWER, std::min(msg.data, VOLTAGE_UPPER));

    if (voltage != msg.data)
        ROS_WARN("Voltage %.2f out of bounds for thrust allocation to PWM conversion. Clamping to [%.2f, %.2f].",
                 msg.data, VOLTAGE_LOWER, VOLTAGE_UPPER);
}

// Given thruster allocation, we reference our last received voltage
// to calculate the appropriate pwm allocation based on voltage and thruster alloc
void Thrusters::thruster_allocs_callback(const custom_msgs::ThrusterAllocs &msg) {

    std::array<uint16_t, NUM_THRUSTERS> pwm_allocs;

    // Set timestamp to now; set up PWMAllocs object to publish
    custom_msgs::PWMAllocs pwm_msg;
    pwm_msg.header.stamp = ros::Time::now();

    // For each of the 8 thrusters in thruster alloc (all of values in -1.0, 1.0)
    // perform a linear interpolation based on the nearest lookup table entries
    for (int i = 0; i < NUM_THRUSTERS; i++) {
        uint16_t alloc = lookup(msg.allocs[i]);
        pwm_allocs[i] = alloc;
        pwm_msg.allocs.push_back(alloc);
    }

    // Write the pwm allocations to the serial port
    write_to_serial(pwm_allocs);

    // Publish the pwm allocations to the /offboard/pwm topic
    pwm_pub.publish(pwm_msg);

}

// Given thruster alloc (force) and the current voltage stored as a field, compute which lookup tables
// to use to perform linear interpolation
double Thrusters::lookup(double force) {
    // Check to make sure force and voltage are not out of bounds
    ROS_ASSERT_MSG(-1.0 <= force && force <= 1.0, "Force must be in [-1.0, 1.0]");
    ROS_ASSERT_MSG(14.0 <= voltage && voltage <= 18.0, "Voltage must be in [14.0, 18.0]");

    // Round force to 2 decimal points (which is what is used to index the lookup tables)
    int index = round_to_two_decimals(force);
    ROS_ASSERT_MSG(0 <= index && index < NUM_LOOKUP_ENTRIES, "Error in PWM lookup. Index %d is out of bounds.", index);

    if (14.0 <= voltage && voltage <= 16.0)
        // Rounded force is between 14.0 and 16.0, use those lookup tables to interpolate
        return interpolate(14.0, v14_lookup_table.at(index), 16.0, v16_lookup_table.at(index), voltage);
    else
        // Rounded force is between 16.0 and 18.0, use those lookup tables to interpolate
        return interpolate(16.0, v16_lookup_table.at(index), 18.0, v18_lookup_table.at(index), voltage);
}

// Perform linear interpolation to compute PWM given force and voltage
double Thrusters::interpolate(double x1, uint16_t y1, double x2, uint16_t y2, double x_interpolate) {
    return y1 + ((y2 - y1) * (x_interpolate - x1)) / (x2 - x1);
}

// Round to two decimal places
int Thrusters::round_to_two_decimals(double num) { return static_cast<int>(std::lround((num * 100) + 100)); }

void Thrusters::write_to_serial(const std::array<uint16_t, NUM_THRUSTERS> &allocs) {

    size_t num_bytes = NUM_THRUSTERS * sizeof(uint16_t) + 1; // include checksum byte

    uint8_t buffer[num_bytes];

    // Copy data (little-endian) to buffer
    for (size_t i = 0; i < NUM_THRUSTERS; i++) {
        buffer[2*i] = allocs[i] & 0xFF;          // lower byte
        buffer[2*i + 1] = (allocs[i] >> 8) & 0xFF; // upper byte
    }

    // Calculate and append checksum with xor
    uint8_t checksum = 0;
    for (size_t i = 0; i < num_bytes - 1; i++) {
        checksum ^= buffer[i];
    }

    buffer[num_bytes - 1] = checksum;

    // Write data + checksum to serial
    ssize_t bytes_written = write(serial_fd, buffer, num_bytes);
    if (bytes_written < 0) {
        ROS_ERROR("Error writing to serial port: %d", errno);
    } else if (bytes_written != num_bytes) {
        ROS_ERROR("Error writing to serial port. Wrote %ld bytes, expected %ld bytes.", bytes_written, num_bytes);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "thrusters");
    ros::NodeHandle nh;

    Thrusters thrusters(argc, argv, nh);

    ros::Rate rate(20);

    while(ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}