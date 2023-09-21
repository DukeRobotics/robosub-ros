#!/usr/bin/env python3

import rospy
import subprocess
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, CompressedImage
from std_msgs.msg import Float64, String
from custom_msgs.msg import CVObject, ThrusterSpeeds

VERBOSE = False

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
        self.sensor_rate = { DVL_ODOM_TOPIC: [], IMU_TOPIC: [], PRESSURE_TOPIC: [],
                             STATE_TOPIC: [], CAMERA_TOPIC: [], SONAR_TOPIC: [],
                             CV_TOPIC: [], OFFBOARD_THRUSTER_SPEEDS_TOPIC: []}
        # Subscribe to all possible sensors
        self.dvl = rospy.Subscriber(DVL_ODOM_TOPIC, Odometry, self.callback, callback_args=DVL_ODOM_TOPIC)
        self.imu = rospy.Subscriber(IMU_TOPIC, Imu, self.callback, callback_args=IMU_TOPIC)
        self.depth_sensor = rospy.Subscriber(PRESSURE_TOPIC, Float64, self.callback, callback_args=PRESSURE_TOPIC)
        self.state = rospy.Subscriber(STATE_TOPIC, Odometry, self.callback, callback_args=STATE_TOPIC)
        self.camera = rospy.Subscriber(CAMERA_TOPIC, CompressedImage, self.callback, callback_args=CAMERA_TOPIC)
        self.sonar = rospy.Subscriber(SONAR_TOPIC, String, self.callback, callback_args=SONAR_TOPIC)
        self.cv_detections = rospy.Subscriber(CV_TOPIC, CVObject, self.callback, callback_args=CV_TOPIC)
        self.thrusters = rospy.Subscriber(OFFBOARD_THRUSTER_SPEEDS_TOPIC, ThrusterSpeeds, self.callback, callback_args=OFFBOARD_THRUSTER_SPEEDS_TOPIC)

    def callback(self, data, topic_name):
        # Record time of response
        self.sensor_rate[topic_name].append(rospy.get_rostime().secs)
        if VERBOSE:
            rospy.loginfo(f"\033[32m{topic_name} has responded!\033[0m")
        # Collected enough samples, report average response gap
        if len(self.sensor_rate[topic_name]) == 5:
            self.echo_status(topic_name)

    def echo_status(self, topic_name):
        # Find the average intervals/gaps in between responses
        response_times = self.sensor_rate[topic_name]
        topic_average = sum([response_times[t] - response_times[t-1] for t in range(1, len(response_times))]) / len(response_times)
        rospy.loginfo(f"\033[32m{topic_name} is up and running...  average gap between responses: {topic_average: .3f}s\033[0m")

    def run(self):
        # Sleep for 5 seconds to collect messages
        rospy.sleep(5)
        rospy.loginfo("5 seconds")
        # Iterate through topics/sensors to report any sensors that aren't active
        for topic_name, responses in self.sensor_rate.items():
            if not responses:
                rospy.logwarn(f"\033[31m {topic_name} is down \033[0m")
            elif 0 < len(responses) < 20:
                rospy.logwarn(f"{topic_name} is responding slow {len(responses)} responses in the last 5 seconds")

if __name__ == "__main__":
    SensorCheckNode().run()
