#!/usr/bin/env python3

import rospy
import rostopic
import yaml
import resource_retriever as rr

from custom_msgs.msg import CVObject
from custom_msgs.srv import EnableModel
from image_tools import ImageTools
from detecto.core import Model


class Detector:
    """This class computes and publishes predictions on a image stream."""

    # Load in models and other misc. setup work
    def __init__(self):
        rospy.init_node('cv', anonymous=True)

        self.image_tools = ImageTools()
        self.camera = rospy.get_param('~camera')

        # Load in model configurations
        with open(rr.get_filename('package://cv/models/models.yaml',
                                  use_protocol=False)) as f:
            self.models = yaml.safe_load(f)

        # The topic that the camera publishes its feed to
        self.camera_feed_topic = f'/camera/{self.camera}/compressed'

        # Toggle model service name
        self.enable_service = f'enable_model_{self.camera}'

    def init_model(self, model_name):
        """Initialize model predictor and publisher if not already initialized.

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

        # Model already initialized; return from method
        if model.get('predictor') is not None:
            return

        weights_file = rr.get_filename(
            f"package://cv/models/{model['weights']}",
            use_protocol=False)

        predictor = Model.load(weights_file, model['classes'])
        publisher_dict = {}

        # Iterate over all classes predicted by a given model,
        # creating new publisher topics for each class
        for model_class in model['classes']:
            publisher_name = f"{model['topic']}/{self.camera}/{model_class}"
            publisher_dict[model_class] = rospy.Publisher(publisher_name,
                                                          CVObject,
                                                          queue_size=10)

        model['predictor'] = predictor
        model['publisher'] = publisher_dict

    def detect(self, img_msg):
        """Compute predictions on a raw image frame and publish results.

        This can be used as the camera topic subscriber callback.

        :param img_msg: ROS Image message to compute predictions on.
        """

        image = self.image_tools.convert_to_cv2(img_msg)

        for model_name in self.models:
            model = self.models[model_name]

            # Generate predictions for each enabled model
            if model.get('enabled'):
                # Initialize predictor if not already
                self.init_model(model_name)

                # Generate model predictions
                labels, boxes, scores = model['predictor'].predict_top(image)

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

    def enable_model(self, req):
        """Service for toggling specific models on and off.

        :param req: The request from another node or command line to enable the model. This service request is
        defined in /robosub-ros/core/catkin_ws/src/custom_msgs/srv/EnableModel.srv
        """

        if req.model_name in self.models:
            model = self.models[req.model_name]
            model['enabled'] = req.enabled

            # Delete model from memory if setting to disabled
            if not model.get('enabled'):
                model['predictor'] = None
                model['publisher'] = None

            return True

        return False

    def run(self):
        """Initialize node and set up Subscriber to generate and publish predictions at every camera frame received."""
        TopicType, _, _ = rostopic.get_topic_class(self.camera_feed_topic)
        rospy.Subscriber(self.camera_feed_topic, TopicType, self.detect)

        # Allow service for toggling of models
        rospy.Service(self.enable_service, EnableModel, self.enable_model)

        # Keep node running until shut down
        rospy.spin()


if __name__ == '__main__':
    try:
        Detector().run()
    except rospy.ROSInterruptException:
        pass
