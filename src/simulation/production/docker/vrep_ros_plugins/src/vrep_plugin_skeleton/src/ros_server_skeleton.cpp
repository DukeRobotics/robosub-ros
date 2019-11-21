#include "../include/vrep_plugin_skeleton/ros_server_skeleton.h"
#include "../include/v_repLib.h"

ros::NodeHandle* ROS_server::node = NULL;

// Services:
ros::ServiceServer ROS_server::displayText_server;

// Publishers:
ros::Publisher ROS_server::objectCount_publisher;

// Subscribers:
ros::Subscriber ROS_server::addStatusBarMessage_subscriber;

bool ROS_server::initialize()
{
    int argc = 0;
    char** argv = NULL;
    ros::init(argc,argv,"vrep");

    if(!ros::master::check())
        return(false);
    
    node=new ros::NodeHandle("~");

    // Enable the services:
    displayText_server = node->advertiseService("displayText",ROS_server::displayText_service);

    // Enable the publishers:
    objectCount_publisher=node->advertise<std_msgs::Int32>("objectCount",1);

    // Enable the subscribers:
    addStatusBarMessage_subscriber=node->subscribe("addStatusbarMessage",1,&ROS_server::addStatusbarMessage_callback);

    return(true);
}

void ROS_server::shutDown()
{
    // Disable the subscribers:
    addStatusBarMessage_subscriber.shutdown();

    // Disable the publishers:
    objectCount_publisher.shutdown();

    // Disable the services:
    displayText_server.shutdown();

    // Shut down:
    ros::shutdown();
}

void ROS_server::instancePass()
{ // When simulation is not running, we "spinOnce" here:
    int simState=simGetSimulationState();
    if ((simState&sim_simulation_advancing)==0)
        spinOnce();
}

void ROS_server::mainScriptAboutToBeCalled()
{ // When simulation is running, we "spinOnce" here:
    spinOnce();
}

void ROS_server::simulationAboutToStart()
{
}

void ROS_server::simulationEnded()
{
}

void ROS_server::spinOnce()
{
    // Disable error reporting (it is enabled in the service processing part, but we don't want error reporting for publishers/subscribers)
    int errorModeSaved;
    simGetIntegerParameter(sim_intparam_error_report_mode,&errorModeSaved);
    simSetIntegerParameter(sim_intparam_error_report_mode,sim_api_errormessage_ignore);

    //Handle all streaming (publishers)
    streamAllData();

    //Process all requested services and topic subscriptions
    ros::spinOnce();

    // Restore previous error report mode:
    simSetIntegerParameter(sim_intparam_error_report_mode,errorModeSaved); 
}

// Services:
bool ROS_server::displayText_service(vrep_skeleton_msg_and_srv::displayText::Request &req,vrep_skeleton_msg_and_srv::displayText::Response &res)
{
    res.dialogHandle=simDisplayDialog("Message from a ROS node",req.textToDisplay.c_str(),sim_dlgstyle_message,NULL,NULL,NULL,NULL);
    return true;
}

// Publishers:
void ROS_server::streamAllData()
{
    // Take care of publishers here (i.e. have them publish their data):
    std_msgs::Int32 objCnt;
    int index=0;
    int h=0;
    while (h>=0)
        h=simGetObjects(index++,sim_handle_all);
    objCnt.data=index-1;
    objectCount_publisher.publish(objCnt);
}

// Subscribers:
void ROS_server::addStatusbarMessage_callback(const std_msgs::String::ConstPtr& msg)
{
    simAddStatusbarMessage(msg->data.c_str());
}
