#ifndef SERIAL_THRUSTERS_H
#define SERIAL_THRUSTERS_H

#include <custom_msgs/PWMAllocs.h>
#include <ros/ros.h>
#include <serial/serial.h>

#define BAUDRATE 57600
#define TIMEOUT 1000

class SerialThrusters {
   private:
    ros::Subscriber pwm_sub;
    serial::Serial* ser;
    void thruster_allocs_callback(const custom_msgs::PWMAllocs &msg);

   public:
    SerialThrusters(int argc, char **argv, ros::NodeHandle &nh);
    ~SerialThrusters();
};

#endif // SERIAL_THRUSTERS_H