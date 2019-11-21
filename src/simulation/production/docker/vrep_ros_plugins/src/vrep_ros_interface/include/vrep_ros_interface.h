#ifndef VREP_ROS_INTERFACE_H_INCLUDED
#define VREP_ROS_INTERFACE_H_INCLUDED

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <map>

#include <boost/lexical_cast.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include "config.h"

#define PLUGIN_NAME "RosInterface"
#define PLUGIN_VERSION 5

struct ScriptCallback
{
    int scriptId;
    std::string name;
};

struct Proxy
{
    bool destroyAfterSimulationStop;
};

#include <ros_msg_builtin_io.h>

struct SubscriberProxy : Proxy
{
    int handle;
    std::string topicName;
    std::string topicType;
    ScriptCallback topicCallback;
    ros::Subscriber subscriber;
    image_transport::Subscriber imageTransportSubscriber;
    WriteOptions wr_opt;
};

struct PublisherProxy : Proxy
{
    int handle;
    std::string topicName;
    std::string topicType;
    ros::Publisher publisher;
    image_transport::Publisher imageTransportPublisher;
    ReadOptions rd_opt;
};

struct ServiceClientProxy : Proxy
{
    int handle;
    std::string serviceName;
    std::string serviceType;
    ros::ServiceClient client;
    ReadOptions rd_opt;
    WriteOptions wr_opt;
};

struct ServiceServerProxy : Proxy
{
    int handle;
    std::string serviceName;
    std::string serviceType;
    ScriptCallback serviceCallback;
    ros::ServiceServer server;
    ReadOptions rd_opt;
    WriteOptions wr_opt;
};

#include <stubs.h>
#include <ros_msg_io.h>
#include <ros_srv_io.h>

#endif // VREP_ROS_INTERFACE_H_INCLUDED
