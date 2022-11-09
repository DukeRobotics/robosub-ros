#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import resource_retriever as rr
from custom_msgs.srv import EnableModel
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class DummyImagePublisher(Node):
    """Mock the camera by publishing a still image to a topic."""

    NODE_NAME = 'test_images'
    CAMERA = 'left'  # TODO: Clean this up so camera and filename are parameters
    IMAGE_TOPIC = f'/camera/{CAMERA}/image_raw'
    SERVICE_NAME = f'enable_model_{CAMERA}'
    RUN_LOOP_RATE = 1

    def __init__(self):
        super().__init__(self.NODE_NAME)
        self.image_publisher = self.create_publisher(Image, self.IMAGE_TOPIC, 10)

        file = rr.get_filename(f'package://cv/assets/left384.jpg', use_protocol=False)
        image = cv2.imread(file, cv2.IMREAD_COLOR)
        bridge = CvBridge()

        self.image_msg = bridge.cv2_to_imgmsg(image, 'bgr8')

        self.enable_model = self.create_client(EnableModel, self.SERVICE_NAME)
        while not self.enable_model.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Service not available, trying again...')

        self.model_enabled = True
        self.count = 0

        self.timer = self.create_timer(1/self.RUN_LOOP_RATE, self.run)

    def run(self):
        """Publish dummy image to topic every second.
        Every 30 frames published, toggle the enable for the prediction model to test the service.
        """
        self.image_publisher.publish(self.image_msg)
        if self.count % 30 == 0:
            req = EnableModel.Request()
            req.model_name = 'gate'
            req.enabled = self.model_enabled
            future = self.enable_model.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            self.model_enabled = not self.model_enabled
        self.count += 1


def main(args=None):
    try:
        rclpy.init(args=args)
        dummy = DummyImagePublisher()
        rclpy.spin(dummy)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        raise
    finally:
        dummy.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
