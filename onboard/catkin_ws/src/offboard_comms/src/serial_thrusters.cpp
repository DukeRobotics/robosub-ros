#include "serial_thrusters.h"

#include <ros/ros.h>
#include <ros/package.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

SerialThrusters::SerialThrusters(int argc, char **argv, ros::NodeHandle &nh) {
    pwm_sub = nh.subscribe("/offboard/pwm", 1, &SerialThrusters::thruster_allocs_callback, this);

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
}

SerialThrusters::~SerialThrusters() {
    if (serial_fd != -1) {
        close(serial_fd);
    }
}

void SerialThrusters::thruster_allocs_callback(const custom_msgs::PWMAllocs &msg) {
    // Write to serial the thruster allocations after receiving them
    std::string write_str = std::to_string(msg.allocs[0]);
    for (int i = 1; i < 8; i++) {
        write_str += "," + std::to_string(msg.allocs[i]);
    }
    write_str += "\n";

    ssize_t bytes_written = write(serial_fd, write_str.c_str(), write_str.size());
    if (bytes_written < 0) {
        ROS_ERROR("Error writing to serial port: %d", errno);
    }

    // Sleep for 10ms to allow the thrusters to process the data
    usleep(10000);
}
