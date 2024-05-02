#include <ros/ros.h>

#include "nonlinear_thrusters.h"
#include "serial_thrusters.h"


int main(int argc, char **argv) {
    ros::init(argc, argv, "thrusters");
    ros::NodeHandle nh;

    NonlinearThrusters thrusters(argc, argv, nh);
    SerialThrusters serial_thrusters(argc, argv, nh);

    ros::spin();
    return 0;
}