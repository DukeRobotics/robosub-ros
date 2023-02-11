#!/usr/bin/env python3

import rospy

from custom_msgs.msg import CVObject
# from custom_msgs.msg import SonarRequest
# from custom_msgs.msg import SonarResponse

class SonarTest:

    def __init__(self):
        """
        Initializes the ROS node and service. Loads the yaml file at cv/models/depthai_models.yaml
        """
        rospy.init_node('sonar_test', anonymous=True)
        self.sonar_requests_publisher = rospy.Publisher("sonar/request", SonarRequest, queue_size=10)

    def request_sonar(self):
        sonar_request_msg = SonarRequest()
        sonar_request_msg.type = "buoy"
        sonar_request_msg.center_degrees = 0
        sonar_request_msg.breadth_degrees = 10
        sonar_request_msg.depth = 5
        self.sonar_requests_publisher.publish(sonar_request_msg)

        result = rospy.wait_for_message("sonar/cv/response", SonarResponse)
        rospy.loginfo(result)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    SonarTest().run()