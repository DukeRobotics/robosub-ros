#ifndef SERIAL_THRUSTERS_H
#define SERIAL_THRUSTERS_H

#include <custom_msgs/PWMAllocs.h>
#include <ros/ros.h>


class SerialThrusters {
   private:
    ros::Subscriber pwm_sub;
    void thruster_allocs_callback(const custom_msgs::PWMAllocs &msg);

    std::string get_serial_port(const std::string &ftdi);
    std::string serial_port;

   public:
    SerialThrusters(int argc, char **argv, ros::NodeHandle &nh);
};

#endif // SERIAL_THRUSTERS_H