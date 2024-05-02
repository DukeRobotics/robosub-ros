#include "serial_thrusters.h"

#include <ros/ros.h>
#include <boost/asio/serial_port.hpp>
#include <boost/asio.hpp>
#include <vector>
#include <dirent.h>
#include <errno.h>


#include "yaml-cpp/yaml.h"


SerialThrusters::SerialThrusters(int argc, char **argv, ros::NodeHandle &nh) {
    pwm_sub = nh.subscribe<custom_msgs::ThrusterAllocs>("thruster_allocs", 1, &SerialThrusters::thruster_allocs_callback, this);

    // Read environment variable to determine which robot we are using
    const char *robot_name = std::getenv("ROBOT_NAME"); // Should be "oogway"

    // Read YAML for proper robot to determine ftdi string for the thruster Arduino
    ROS_ASSERT_MSG(robot_name != NULL, "ROBOT_NAME environment variable not set");

    std::string yaml_path = ros::package::getPath("offboard_comms") + "/config/" + robot_name + ".yaml";

    ROS_LOG("Reading YAML file %s", yaml_path.c_str());

    // Get the serial port for the thruster Arduino
    YAML::Node config = YAML::LoadFile(yaml_path);
    std::string ftdi = config["arduino"]["thruster"]["ftdi"].as<std::string>();

    // Get the serial port that corresponds to the ftid string
    serial_port = get_serial_port(ftdi);
}

std::string SerialThrusters::get_serial_port(const std::string &ftdi) {
    // Get a list of all serial ports on the system
    std::vector<std::string> ports;

    // Fill the ports vector with the available serial ports
    DIR *dir;
    struct dirent *ent;
    if ((dir = opendir("/dev")) != NULL) {
        while((ent = readdir(dir)) != NULL) {
            std::string port_name = ent->d_name;
            if(port_name.find("ttyUSB") != std::string::npos || port_name.find("ttyACM") != std::string::npos) {
                ports.push_back("/dev/" + port_name);
            }
        }
        closedir(dir);
    } else {
        // Could not open directory
        ROS_ERROR("Could not open /dev directory: %s", strerror(errno));
        return "";
    }

    // Iterate over the ports and find the one that matches the ftdi string
    for (const auto &port_name : ports) {
        boost::asio::io_service io;
        boost::asio::serial_port port(io);
        port.open(port_name);
        if (port.is_open() && port.get_option(boost::asio::serial_port_base::baud_rate()).value() == ftdi) {
            // Return the name of the matching port
            return port_name;
        }
    }

    // If no matching port was found, return an empty string
    ROS_ERROR("Could not find serial port for ftdi %s", ftdi.c_str());
    return "";
}


void SerialThrusters::thruster_allocs_callback(const custom_msgs::ThrusterAllocs &msg) {
    // Write to serial the thruster allocations after we receive them
    // Since the thrusters store the previous values, we do not need to store them here
    // We just need to write the values to the serial port

    // Write to serial
    boost::asio::io_service io;
    boost::asio::serial_port port(io);
    port.open(serial_port);
    if (!port.is_open()) {
        ROS_ERROR("Could not open serial port %s", serial_port.c_str());
        return;
    }

    // Write the thruster allocations to the serial port as comma separated values
    // TODO
}