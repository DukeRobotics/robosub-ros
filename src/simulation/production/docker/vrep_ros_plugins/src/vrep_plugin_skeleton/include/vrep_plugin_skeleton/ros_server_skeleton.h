#ifndef ROS_SERVER_H
#define ROS_SERVER_H

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

// API services:
#include "vrep_skeleton_msg_and_srv/displayText.h"

class ROS_server
{
    public:
        static bool initialize();
        static void shutDown();

        static void instancePass();
        static void mainScriptAboutToBeCalled();

        static void simulationAboutToStart();
        static void simulationEnded();

    private:
        ROS_server() {}; 
        
        static ros::NodeHandle* node;

        static void spinOnce();

        // Services:
        static bool displayText_service(vrep_skeleton_msg_and_srv::displayText::Request &req,vrep_skeleton_msg_and_srv::displayText::Response &res);
        static ros::ServiceServer displayText_server;

        // Publishers:
        static void streamAllData();
        static ros::Publisher objectCount_publisher;

        // Subscribers:
        static void addStatusbarMessage_callback(const std_msgs::String::ConstPtr& msg);
        static ros::Subscriber addStatusBarMessage_subscriber;
};

#endif
