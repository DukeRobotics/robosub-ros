#include "serial_thrusters.h"

#include <ros/ros.h>
#include <ros/package.h>
#include <boost/asio/serial_port.hpp>
#include <boost/asio.hpp>
#include <vector>
#include <dirent.h>
#include <errno.h>
#include <libudev.h>

#include "yaml-cpp/yaml.h"


SerialThrusters::SerialThrusters(int argc, char **argv, ros::NodeHandle &nh) {
    // pwm_sub = nh.subscribe<custom_msgs::ThrusterAllocs>("thruster_allocs", 1, &SerialThrusters::thruster_allocs_callback, this);

    // Read environment variable to determine which robot we are using
    const char *robot_name = std::getenv("ROBOT_NAME"); // Should be "oogway"

    // Read YAML for proper robot to determine ftdi string for the thruster Arduino
    ROS_ASSERT_MSG(robot_name != NULL, "ROBOT_NAME environment variable not set");

    std::string yaml_path = ros::package::getPath("offboard_comms") + "/config/" + robot_name + ".yaml";

    ROS_INFO("Reading from %s", yaml_path.c_str());

    // Get the serial port for the thruster Arduino
    YAML::Node config = YAML::LoadFile(yaml_path);
    std::string ftdi = config["arduino"]["thruster"]["ftdi"].as<std::string>();

    ROS_INFO("Thruster FTDI: %s", ftdi.c_str());

    // Get the serial port that corresponds to the ftid string
    serial_port = get_serial_port(ftdi);
}

std::string get_serial_number(const std::string& port_name) {
    struct udev *udev;
    struct udev_device *dev;
    const char *serial;

    // Create a new udev object
    udev = udev_new();
    if (!udev) {
        ROS_ERROR("Can't create udev");
        return "";
    }

    // Get a device object for the given port
    dev = udev_device_new_from_subsystem_sysname(udev, "tty", port_name.c_str());
    if (!dev) {
        ROS_ERROR("Failed to get device: %s", port_name.c_str());
        udev_unref(udev);
        return "";
    }

    // Get the parent device, with the subsystem "usb"
    dev = udev_device_get_parent_with_subsystem_devtype(dev, "usb", "usb_device");
    if (!dev) {
        ROS_ERROR("Unable to find parent usb device of %s", port_name.c_str());
        udev_unref(udev);
        return "";
    }

    // Get the serial number
    serial = udev_device_get_sysattr_value(dev, "serial");
    if (!serial) {
        ROS_ERROR("Unable to get serial number of %s", port_name.c_str());
        udev_unref(udev);
        return "";
    }

    // Clean up and return the serial number
    udev_unref(udev);
    return std::string(serial);
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

        // Get the serial number of the port
        boost::asio::io_service io;
        boost::asio::serial_port port(io);
        port.open(port_name);
        if (!port.is_open()) {
            ROS_ERROR("Could not open serial port %s", port_name.c_str());
            continue;
        }

        ROS_INFO("Opened port %s", port_name.c_str());

        // Get the serial number of device
        std::string serial = get_serial_number(port_name);
        if (serial == ftdi) {
            ROS_INFO("Found matching port %s", port_name.c_str());
            return port_name;
        }
    }

    // If no matching port was found, return an empty string
    ROS_ERROR("Could not find serial port for ftdi %s", ftdi.c_str());
    return "";
}


// void SerialThrusters::thruster_allocs_callback(const custom_msgs::ThrusterAllocs &msg) {
//     // Write to serial the thruster allocations after we receive them
//     // Since the thrusters store the previous values, we do not need to store them here
//     // We just need to write the values to the serial port

//     // Write to serial
//     boost::asio::io_service io;
//     boost::asio::serial_port port(io);
//     port.open(serial_port);
//     if (!port.is_open()) {
//         ROS_ERROR("Could not open serial port %s", serial_port.c_str());
//         return;
//     }

//     // Write the thruster allocations to the serial port as comma separated values
//     // TODO
// }