#!/usr/bin/env python3

import rospy
import subprocess
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, CompressedImage
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float64, String
from custom_msgs.msg import ThrusterSpeeds

VERBOSE = False

# Dictionary of all sensor topics and types: (TOPIC_NAME, MESSAGE_TYPE)
SENSOR_SUBSCRIBE_TOPICS = {'/sensors/dvl/odom': Odometry,
                           '/vectornav/IMU': Imu,
                           '/sensors/depth': PoseWithCovarianceStamped,
                           '/state': Odometry,
                           '/camera/front/rgb/preview/compressed': CompressedImage,
                           '/sonar/status': String}

OFFBOARD_THRUSTER_POWER_TOPIC = '/offboard/thruster_speeds'

class SensorCheckNode:

    def __init__(self):

        rospy.init_node('sensor_check')
        rospy.loginfo('Would you like to run the thrusters? 1 for yes, 0 for no')
        self.test_thrusters = int(input())

        # Subscribe to all possible sensors
        self.sensor_subscibers = dict()
        self.sensor_rate = dict()
        for topic, message_type in SENSOR_SUBSCRIBE_TOPICS.items():
            self.sensor_subscibers[topic] = rospy.Subscriber(topic, message_type, self.callback, callback_args=topic)
            self.sensor_rate[topic] = []
        
        # Publish to offboard/thrusters and run thrusters at low speeds if test_thrusters is True
        if self.test_thrusters == 1:
            rospy.loginfo("Testing thrusters...")
            self.thruster_tester = rospy.Publisher(OFFBOARD_THRUSTER_POWER_TOPIC, ThrusterSpeeds, queue_size=10)
            self.spin_thrusters_at_low_speeds()

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
        topic_average = sum([response_times[t] - response_times[t-1]
                             for t in range(1, len(response_times))]) / len(response_times)
        rospy.loginfo(f"\033[32m{topic_name} is up and running... \
                      average gap between responses: {topic_average: .3f}s\033[0m")

    def spin_thrusters_at_low_speeds(self):
        # Spin for 5 seconds
        desired_speed = ThrusterSpeeds()
        desired_speed.speeds = [20, 20, 20, 20, 20, 20, 20, 20]
        publishing_rate = rospy.Rate(10)
        curr_time = rospy.get_rostime().secs
        while not rospy.is_shutdown():
            self.thruster_tester.publish(desired_speed)
            publishing_rate.sleep()
            now = rospy.get_rostime().secs
            if now - curr_time >= 5:
                break

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
