#!/usr/bin/env python3

import rospy
import time

from custom_msgs.msg import CVObject
from custom_msgs.msg import sweepGoal, sweepResult

class SonarTest:

    def __init__(self):
        rospy.init_node('sonar_test', anonymous=True)
        self.sonar_requests_publisher = rospy.Publisher("sonar/request", sweepGoal, queue_size=10)
        sonar_request_msg = sweepGoal()
        sonar_request_msg.distance_of_scan = -1
        self.sonar_requests_publisher.publish(sonar_request_msg)
        self.x_pos = 0
        self.y_pos = 0
        self.sonar_requests_subscriber = rospy.Subscriber("sonar/cv/response", sweepResult, self.updatePos)

    def request_sonar(self):
        sonar_request_msg = sweepGoal()
        sonar_request_msg.start_angle = 195
        sonar_request_msg.end_angle = 205
        sonar_request_msg.distance_of_scan = 5

        start_time = time.perf_counter()
        self.sonar_requests_publisher.publish(sonar_request_msg)
        result = (self.x_pos, self.y_pos)

        end_time = time.perf_counter()
        delta_time = end_time - start_time
        # rospy.loginfo("Delta time: ", delta_time)

        if not (result == (0, 0)):
            rospy.loginfo(result)

    def updatePos(self, result):
        self.x_pos = result.x_pos
        self.y_pos = result.y_pos

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    st = SonarTest()
    i = 0
    while True:
        st.request_sonar()
        time.sleep(1)
        i+=1
