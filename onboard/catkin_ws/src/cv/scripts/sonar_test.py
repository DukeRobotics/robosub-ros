#!/usr/bin/env python3

import rospy
import time

from custom_msgs.msg import CVObject
from custom_msgs.msg import sweepGoal, sweepResult

class SonarTest:

    def __init__(self):
        rospy.init_node('sonar_test', anonymous=True)
        self.sonar_requests_publisher = rospy.Publisher("sonar/request", sweepGoal, queue_size=10)

    def request_sonar(self):
        sonar_request_msg = sweepGoal()
        # sonar_request_msg.type = "buoy"
        sonar_request_msg.start_angle = 195
        sonar_request_msg.end_angle = 205
        sonar_request_msg.distance_of_scan = 5

        start_time = time.perf_counter()
        rospy.loginfo("hello")
        self.sonar_requests_publisher.publish(sonar_request_msg)
        rospy.loginfo("published request")

        result = rospy.wait_for_message("sonar/cv/response", sweepResult)
        end_time = time.perf_counter()
        delta_time = end_time - start_time
        rospy.loginfo(delta_time)
        rospy.loginfo(result)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    st = SonarTest()
    st.request_sonar()
    time.sleep(2)
    st.request_sonar()
    time.sleep(2)
    st.request_sonar()