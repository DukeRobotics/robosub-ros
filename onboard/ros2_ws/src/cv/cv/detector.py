#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import yaml
import resource_retriever as rr

from custom_msgs.msg import CVObject
from custom_msgs.srv import EnableModel
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from detecto.core import Model


class Detector(Node):
    """This class computes and publishes predictions on a image stream."""

    NODE_NAME = 'detector'

    # Load in models and other misc. setup work
    def __init__(self):
        super().__init__(self.NODE_NAME)

        self.bridge = CvBridge()
        self.camera = self.declare_parameter('camera', 'left').value
        # Load in model configurations
        with open(rr.get_filename('package://cv/models/models.yaml', use_protocol=False)) as f:
            self.models = yaml.safe_load(f)

        for model_name in self.models:
            self.models[model_name]['predictor'] = None
            self.models[model_name]['publisher'] = None

        camera_feed_topic = f'/camera/{self.camera}/image_raw'
        enable_service = f'enable_model_{self.camera}'
        self.create_subscription(Image, camera_feed_topic, self.detect, 10)
        self.create_service(EnableModel, enable_service, self.enable_model)

    def init_model(self, model_name):
        """Initialize model predictor and publisher.

        There will be a single topic for every class. The format for the topics will be cv/<camera>/<class-name>

        :param model_name: The name of the model to initialize. This string should be a key in the
        cv/models/models.yaml file. For example, if the models.yaml file is:

        gate:
            classes: [gate]
            topic: /cv
            weights: detect-gate.pth

        and you would like to initialize the gate model, the model_name should be 'gate'
        """

        model = self.models[model_name]

        weights_file = rr.get_filename(
            f"package://cv/models/{model['weights']}",
            use_protocol=False)

        predictor = Model.load(weights_file, model['classes'])
        publisher_dict = {}

        # Iterate over all classes predicted by a given model,
        # creating new publisher topics for each class
        for model_class in model['classes']:
            publisher_name = f"{model['topic']}/{self.camera}/{model_class}"
            publisher_dict[model_class] = self.create_publisher(CVObject, publisher_name, 10)

        model['predictor'] = predictor
        model['publisher'] = publisher_dict

    def detect(self, img_msg):
        """Compute predictions on a raw image frame and publish results.

        This can be used as the camera topic subscriber callback.

        :param img_msg: ROS Image message to compute predictions on.
        """

        image = self.bridge.imgmsg_to_cv2(img_msg, 'rgb8')

        for model_name in self.models:
            model = self.models[model_name]
            # Generate predictions for each enabled model
            if model.get('enabled'):
                if model.get('predictor') is None:
                    self.init_model(model_name)
                # Generate model predictions
                labels, boxes, scores = model['predictor'].predict_top(image)

                # Pass raw model predictions into nms algorithm for filtering
                # nms_labels, nms_boxes, nms_scores = utils.nms(labels, boxes,
                #                                               scores.detach())

                # Publish post-nms predictions
                self.publish_predictions((labels, boxes, scores),
                                         model['publisher'], image.shape)

    def publish_predictions(self, preds, publisher, shape):
        """Publish prediction results to a publisher based on which predicted class each object is.

        :param preds: Tuple of labels, bounding boxes, and scores. These are returned from detecto's
        predict_top() function. For futher information, see https://detecto.readthedocs.io/en/latest/api/core.html#.
        :param publisher: Dictionary of publishers for the model. keys are the class / label string and the values are
        the publisher for that class.
        :param shape: The shape of the image in format (height, width)
        """

        labels, boxes, scores = preds

        # If there are no predictions, publish nothing
        if not labels:
            object_msg = CVObject()
            object_msg.label = 'none'
        else:
            for label, box, score in zip(labels, boxes, scores):
                object_msg = CVObject()

                object_msg.label = label
                object_msg.score = score

                object_msg.xmin = box[0].item() / shape[1]
                object_msg.ymin = box[1].item() / shape[0]
                object_msg.xmax = box[2].item() / shape[1]
                object_msg.ymax = box[3].item() / shape[0]

                object_msg.height = shape[0]
                object_msg.width = shape[1]

                # Safety check that publisher is not None
                if publisher:
                    # Publish to the publisher topic corresponding to
                    # the given returned label
                    publisher[label].publish(object_msg)

    def enable_model(self, req, res):
        """Service for toggling specific models on and off.

        :param req: The request from another node or command line to enable the model. This service request is
        defined in /robosub-ros/core/catkin_ws/src/custom_msgs/srv/EnableModel.srv
        """
        res.success = False
        if req.model_name in self.models:
            model = self.models[req.model_name]
            model['enabled'] = req.enabled

            # Delete model from memory if setting to disabled
            if not model.get('enabled'):
                model['predictor'] = None
                model['publisher'] = None

            res.success = True

        return res


def main(args=None):
    try:
        rclpy.init(args=args)
        detector = Detector()
        rclpy.spin(detector)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        raise
    finally:
        detector.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
