#ifndef SERIAL_THRUSTERS_H
#define SERIAL_THRUSTERS_H

#include <custom_msgs/PWMAllocs.h>
#include <ros/ros.h>

class SerialThrusters {
   private:
    ros::Subscriber pwm_sub;
    int serial_fd;
    void thruster_allocs_callback(const custom_msgs::PWMAllocs &msg);

   public:
    SerialThrusters(int argc, char **argv, ros::NodeHandle &nh);
    ~SerialThrusters();
};

#endif // SERIAL_THRUSTERS_H