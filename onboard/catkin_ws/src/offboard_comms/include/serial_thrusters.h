#ifndef SERIAL_THRUSTERS_H
#define SERIAL_THRUSTERS_H

#include <custom_msgs/PWMAllocs.h>
#include <ros/ros.h>
#include <boost/asio.hpp>

class SerialThrusters {
   private:
    ros::Subscriber pwm_sub;
    void thruster_allocs_callback(const custom_msgs::PWMAllocs &msg);

    std::string get_serial_port(const std::string &ftdi);
    std::string serial_port;

    boost::asio::serial_port port;

   public:
    SerialThrusters(boost::asio::io_service& io, int argc, char **argv, ros::NodeHandle &nh);
    ~SerialThrusters();
};

#endif // SERIAL_THRUSTERS_H