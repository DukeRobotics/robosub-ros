#include "serial_thrusters.h"

#include <ros/ros.h>
#include <ros/package.h>


SerialThrusters::SerialThrusters(int argc, char **argv, ros::NodeHandle &nh) {
    pwm_sub = nh.subscribe("/offboard/pwm", 1, &SerialThrusters::thruster_allocs_callback, this);


    std::string device;
    if (!nh.getParam("device", device)) {
        ROS_ERROR("Failed to get parameter 'device'");
        return;
    }

    // Validate the device path
    if (device.find("/dev/") == std::string::npos) {
        ROS_ERROR("Invalid device path: %s", device.c_str());
        return;
    }

    ROS_INFO("Thruster serial port: %s", device.c_str());

    // Open the serial port, retrying every 5 seconds if it fails
    do {
        try {
            ser = new serial::Serial(device, BAUDRATE, serial::Timeout::simpleTimeout(TIMEOUT));
            ser->open();
            break;
        } catch (serial::SerialException &e) {
            ROS_ERROR("Failed to open serial port: %s", e.what());
        } catch (serial::IOException &e) {
            ROS_ERROR("IO exception when opening serial port: %s", e.what());
        }

        ROS_ERROR("Retrying in 5 seconds");
        for (int i = 0; i < 50 && ros::ok(); ++i) {
            ros::Duration(0.1).sleep();
        }
    } while (ros::ok());

    ROS_INFO("Thrusters initialized");
}

SerialThrusters::~SerialThrusters() {
    ser->close();
    delete ser;
}

void SerialThrusters::thruster_allocs_callback(const custom_msgs::PWMAllocs &msg) {
    // Write to serial the thruster allocations after we receive them
    // Since the thrusters store the previous values, we do not need to store them here
    // We just need to write the new values to the serial port


    // Write as comma separated values
    std::string write_str = std::to_string(msg.allocs[0]);
    for (int i = 1; i < 8; i++) {
        write_str += "," + std::to_string(msg.allocs[i]);
    }

    write_str += "\n";

    ROS_INFO("Writing to serial: %s", write_str.c_str());

    if (ser->isOpen()) {
        ser->write(write_str);
    } else {
        ROS_ERROR("Serial port is not open");
    }

    // Sleep for 10ms to allow the thrusters to process the data
    usleep(10000);
}