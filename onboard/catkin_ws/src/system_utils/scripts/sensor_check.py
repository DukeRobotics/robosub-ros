#!/usr/bin/env python3

import rospy
import subprocess
from nav_msgs.msg import Odometry
from sensor_msg import Imu, CompressedImage
from std_msgs.msg import Float64, String
from custom_msgs.msg import CVObject, ThrusterSpeeds

DVL_ODOM_TOPIC = 'sensors/dvl/odom'
IMU_TOPIC = '/vectornav/IMU'
PRESSURE_TOPIC = '/sensors/depth'
STATE_TOPIC = '/state'
CAMERA_TOPIC = '/camera/front/rgb/preview/compressed'
SONAR_TOPIC = 'sonar/status'

CV_TOPIC = '/cv/front/buoy_earth_cetus'
OFFBOARD_THRUSTER_SPEEDS_TOPIC = '/offboard/thruster_speeds'

class SensorCheckNode:

    def __init__(self):
        rospy.init_node('sensor_check')
        self.sensor_rate = { DVL_ODOM_TOPIC: 0, IMU_TOPIC: 0, PRESSURE_TOPIC: 0,
                             STATE_TOPIC: 0, CAMERA_TOPIC: 0, SONAR_TOPIC: 0,
                             CV_TOPIC: 0, OFFBOARD_THRUSTER_SPEEDS_TOPIC: 0}
        # Subscribe to all possible sensors
        self.dvl = rospy.Subscriber(DVL_ODOM_TOPIC, Odometry, self.callback(DVL_ODOM_TOPIC))
        self.imu = rospy.Subscriber(IMU_TOPIC, Imu, self.callback(IMU_TOPIC))
        self.depth_sensor = rospy.Subscriber(PRESSURE_TOPIC, Float64, self.callback(PRESSURE_TOPIC))
        self.state = rospy.Subscriber(STATE_TOPIC, Odometry, self.callback(STATE_TOPIC))
        self.camera = rospy.Subscriber(CAMERA_TOPIC, CompressedImage, self.callback(CAMERA_TOPIC))
        self.sonar = rospy.Subscriber(SONAR_TOPIC, String, self.callback(SONAR_TOPIC))
        self.cv_detections = rospy.Subscriber(CV_TOPIC, CVObject, self.callback(CV_TOPIC))
        self.thrusters = rospy.Subscriber(OFFBOARD_THRUSTER_SPEEDS_TOPIC, ThrusterSpeeds, self.callback(OFFBOARD_THRUSTER_SPEEDS_TOPIC))

    def callback(self, topic_name):
        self.sensor_rate[topic_name] += 1

if __name__ == "__main__":
    SensorCheckNode()