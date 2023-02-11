
import rospy
from sonar import Sonar
from custom_msgs.msg import sweepResult, sweepGoal
from sonar_utils import degrees_to_centered_gradians

class SonarPublisher:

    SONAR_REQUEST_TOPIC = 'sonar/request'
    SONAR_RESPONSE_TOPIC = 'sonar/cv/response'

    NODE_NAME = "sonar_pub"

    SONAR_DEFAULT_RANGE = 5

    def __init__(self):
        self.sonar = Sonar(5)
        self._pub_request = rospy.Publisher(self.SONAR_RESPONSE_TOPIC, sweepGoal, queue_size=10)

    def on_request(self, request):
        self.sonar.set_new_range(request.distance_of_scan)
        # center_gradians = degrees_to_centered_gradians(request.center_degrees)
        # breadth_gradians = degrees_to_centered_gradians(request.breadth_degrees)

        # left = max(center_gradians - breadth_gradians, 0)
        # right = min(center_gradians + breadth_gradians, 400)

        sonar_xy_result = self.sonar.get_xy_of_object_in_sweep(request.start_angle,request.end_angle)

        response = sweepResult()
        response.x_pos = sonar_xy_result[0]
        response.y_pos = sonar_xy_result[1]
        self._pub_request.publish(response)
        

    def run(self):
        rospy.init_node(self.NODE_NAME)
        rospy.Subscriber(self.SONAR_REQUEST_TOPIC, sweepGoal, self.on_request)
        rospy.spin()


if __name__ == '__main__':
    try:
        SonarPublisher().run()
    except rospy.ROSInterruptException:
        pass
