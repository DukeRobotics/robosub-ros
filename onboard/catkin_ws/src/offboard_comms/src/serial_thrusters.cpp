#include "serial_thrusters.h"

#include <ros/ros.h>
#include <ros/package.h>
#include <dirent.h>
#include <unistd.h>
#include <boost/asio.hpp>

#include "yaml-cpp/yaml.h"


SerialThrusters::SerialThrusters(int argc, char **argv, ros::NodeHandle &nh) {
    pwm_sub = nh.subscribe("/offboard/pwm", 1, &SerialThrusters::thruster_allocs_callback, this);

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

    ROS_INFO("Thruster serial port: %s", serial_port.c_str());
}



std::string SerialThrusters::get_serial_port(const std::string &ftdi) {
    DIR* dir;
    struct dirent* ent;
    char target_path[1024];

    if ((dir = opendir("/dev/serial/by-id/")) != NULL) {
        while ((ent = readdir(dir)) != NULL) {
            if (strstr(ent->d_name, ftdi.c_str()) != NULL) {
                std::string full_path = "/dev/serial/by-id/" + std::string(ent->d_name);
                ssize_t len = readlink(full_path.c_str(), target_path, sizeof(target_path)-1);
                if (len != -1) {
                    target_path[len] = '\0';
                    closedir(dir);
                    // Slice off up to the /
                    std::string path_str(target_path);
                    size_t pos = path_str.find_last_of("/");
                    return "/dev/" + path_str.substr(pos + 1);
                }
            }
        }
        closedir(dir);
    }

    ROS_ERROR("Could not find serial port for FTDI %s", ftdi.c_str());
    return "";
}

void SerialThrusters::thruster_allocs_callback(const custom_msgs::PWMAllocs &msg) {
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

    port.set_option(boost::asio::serial_port_base::baud_rate(57600));
    port.set_option(boost::asio::serial_port_base::character_size(8));
    port.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    port.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));

    // Write as comma separated values
    std::string write_str = std::to_string(msg.allocs[0]);
    for (int i = 1; i < 8; i++) {
        write_str += "," + std::to_string(msg.allocs[i]);
    }

    write_str += "\n";

    ROS_INFO("Writing to serial: %s", write_str.c_str());
    std::size_t bytes_written =  boost::asio::write(port, boost::asio::buffer(write_str.c_str(), write_str.size()));

    if (bytes_written != write_str.size()) {
        ROS_ERROR("Could not write to serial port %s", serial_port.c_str());
    }

    port.close();

    ROS_INFO("Wrote to serial");

    //Delay for 100ms to allow the thrusters to process the data
    usleep(100000);


}