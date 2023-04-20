#!/usr/bin/env python3

import rospy
from sonar import Sonar
from custom_msgs.msg import sweepResult, sweepGoal
from sonar_utils import degrees_to_centered_gradians


class SonarPublisher:

    SONAR_REQUEST_TOPIC = 'sonar/request'
    SONAR_RESPONSE_TOPIC = 'sonar/cv/response'

    NODE_NAME = "sonar_pub"

    SONAR_DEFAULT_RANGE = 10

    def __init__(self):
        rospy.init_node(self.NODE_NAME)
        self.sonar = Sonar(10)
        self._pub_request = rospy.Publisher(self.SONAR_RESPONSE_TOPIC,
                                            sweepResult, queue_size=10)

    def on_request(self, request):
        if (request.distance_of_scan == -1):
            return
        self.sonar.set_new_range(request.distance_of_scan)

        left_gradians = degrees_to_centered_gradians(request.start_angle)
        right_gradians = degrees_to_centered_gradians(request.end_angle)

        sonar_xy_result = self.sonar.get_xy_of_object_in_sweep(left_gradians,
                                                               right_gradians)

        response = sweepResult()
        response.x_pos = sonar_xy_result[0]
        response.y_pos = sonar_xy_result[1]
        self._pub_request.publish(response)

    def run(self):
        rospy.Subscriber(self.SONAR_REQUEST_TOPIC, sweepGoal, self.on_request)
        rospy.loginfo("starting sonar_publisher...")
        rospy.spin()


if __name__ == '__main__':
    try:
        SonarPublisher().run()
    except rospy.ROSInterruptException:
        pass
