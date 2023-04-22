
import rospy
from sonar import Sonar
from custom_msgs.msg import sweepResult, sweepGoal
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from sonar_utils import degrees_to_centered_gradians
from sonar_image_processing import build_sonar_image


class SonarPublisher:

    SONAR_REQUEST_TOPIC = 'sonar/request'
    SONAR_RESPONSE_TOPIC = 'sonar/cv/response'
    SONAR_IMAGE_TOPIC = 'sonar/image'

    NODE_NAME = "sonar_pub"

    SONAR_DEFAULT_RANGE = 10

    def __init__(self):
        rospy.init_node(self.NODE_NAME)
        self.stream = rospy.get_param('~stream')
        self.port = rospy.get_param('~port')
        self.sonar = Sonar(10, serial_port_number=self.port)
        self.cv_bridge = CvBridge()
        self._pub_request = rospy.Publisher(self.SONAR_RESPONSE_TOPIC,
                                            sweepResult, queue_size=10)
        self.sonar_image_publisher = rospy.Publisher(self.SONAR_IMAGE_TOPIC,
                                                     CompressedImage, queue_size=10)

    def on_request(self, request):
        if (request.distance_of_scan == -1):
            return
        self.sonar.set_new_range(request.distance_of_scan)

        left_gradians = degrees_to_centered_gradians(request.start_angle)
        right_gradians = degrees_to_centered_gradians(request.end_angle)

        sonar_xy_result, scanned_image = self.sonar.get_xy_of_object_in_sweep(left_gradians,
                                                                              right_gradians)

        if self.stream:
            sonar_image = build_sonar_image(scanned_image)
            compressed_image = self.image_tools.convert_to_ros_compressed_msg(sonar_image)
            self.sonar_image_publisher.publish(compressed_image)

        response = sweepResult()
        response.x_pos = sonar_xy_result[0]
        response.y_pos = sonar_xy_result[1]
        self._pub_request.publish(response)

    def convert_to_ros_compressed_msg(self, image, compressed_format='jpg'):
        """
        Convert any kind of image to ROS Compressed Image.
        """
        return self.cv_bridge.cv2_to_compressed_imgmsg(image, dst_format=compressed_format)

    def run(self):
        rospy.Subscriber(self.SONAR_REQUEST_TOPIC, sweepGoal, self.on_request)
        rospy.loginfo("starting sonar_publisher...")
        rospy.spin()


if __name__ == '__main__':
    try:
        SonarPublisher().run()
    except rospy.ROSInterruptException:
        pass
