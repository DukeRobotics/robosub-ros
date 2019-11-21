#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"

// Global variables (also modified by the topic subscriber):
bool sensorTrigger=false;
struct timeval tv;
unsigned int currentTime_updatedByTopicSubscriber=0;
float simulationTime=0.0;

// Topic subscriber callbacks:
void sensorCallback(const std_msgs::Bool& sensTrigger)
{
    if (gettimeofday(&tv,NULL)==0)
        currentTime_updatedByTopicSubscriber=tv.tv_sec;
    sensorTrigger=sensTrigger.data;
}

void simulationTimeCallback(const std_msgs::Float32& simTime)
{
    simulationTime=simTime.data;
}

// Main code:
int main(int argc,char* argv[])
{
    // The robot motor velocities and the sensor topic names are given in the argument list
    // (when V-REP launches this executable, V-REP will also provide the argument list)
    std::string leftMotorTopic;
    std::string rightMotorTopic;
    std::string sensorTopic;
    std::string simulationTimeTopic;
    if (argc>=5)
    {
        leftMotorTopic=argv[1];
        rightMotorTopic=argv[2];
        sensorTopic=argv[3];
        simulationTimeTopic=argv[4];
        leftMotorTopic="/"+leftMotorTopic;
        rightMotorTopic="/"+rightMotorTopic;
        sensorTopic="/"+sensorTopic;
        simulationTimeTopic="/"+simulationTimeTopic;
    }
    else
    {
        printf("Indicate following arguments: 'leftMotorTopic rightMotorTopic sensorTopic simulationTimeTopic'!\n");
        sleep(5000);
        return 0;
    }

    // Create a ROS node. The name has a random component: 
    int _argc = 0;
    char** _argv = NULL;
    if (gettimeofday(&tv,NULL)==0)
        currentTime_updatedByTopicSubscriber=(tv.tv_sec*1000+tv.tv_usec/1000)&0x00ffffff;
    std::string nodeName("rosBubbleRob");
    std::string randId(boost::lexical_cast<std::string>(currentTime_updatedByTopicSubscriber+int(999999.0f*(rand()/(float)RAND_MAX))));
    nodeName+=randId;       
    ros::init(_argc,_argv,nodeName.c_str());

    if(!ros::master::check())
        return(0);
    
    ros::NodeHandle node("~");  
    printf("rosBubbleRob2 just started with node name %s\n",nodeName.c_str());

    // 1. Let's subscribe to the sensor and simulation time stream
    ros::Subscriber subSensor=node.subscribe(sensorTopic.c_str(),1,sensorCallback);
    ros::Subscriber subSimulationTime=node.subscribe(simulationTimeTopic.c_str(),1,simulationTimeCallback);

    // 2. Let's prepare publishers for the motor speeds:
    ros::Publisher leftMotorSpeedPub=node.advertise<std_msgs::Float32>(leftMotorTopic.c_str(),1);
    ros::Publisher rightMotorSpeedPub=node.advertise<std_msgs::Float32>(rightMotorTopic.c_str(),1);

    // 3. Finally we have the control loop:
    float driveBackStartTime=-99.0f;
    unsigned int currentTime;
    if (gettimeofday(&tv,NULL)==0)
    {
        currentTime_updatedByTopicSubscriber=tv.tv_sec;
        currentTime=currentTime_updatedByTopicSubscriber;
    }
    while (ros::ok())
    { // this is the control loop (very simple, just as an example)
        if (gettimeofday(&tv,NULL)==0)
        {
            currentTime=tv.tv_sec;
            if (currentTime-currentTime_updatedByTopicSubscriber>9)
                break; // we didn't receive any sensor information for quite a while... we leave
        }
        float desiredLeftMotorSpeed;
        float desiredRightMotorSpeed;
        if (simulationTime-driveBackStartTime<3.0f)
        { // driving backwards while slightly turning:
            desiredLeftMotorSpeed=-3.1415*0.5;
            desiredRightMotorSpeed=-3.1415*0.25;
        }
        else
        { // going forward:
            desiredLeftMotorSpeed=3.1415;
            desiredRightMotorSpeed=3.1415;
            if (sensorTrigger)
                driveBackStartTime=simulationTime; // We detected something, and start the backward mode
            sensorTrigger=false;
        }

        // publish the motor speeds:
        std_msgs::Float32 d;
        d.data=desiredLeftMotorSpeed;
        leftMotorSpeedPub.publish(d);
        d.data=desiredRightMotorSpeed;
        rightMotorSpeedPub.publish(d);

        // handle ROS messages:
        ros::spinOnce();

        // sleep a bit:
        usleep(5000);
    }
    ros::shutdown();
    printf("rosBubbleRob2 just ended!\n");
    return(0);
}

