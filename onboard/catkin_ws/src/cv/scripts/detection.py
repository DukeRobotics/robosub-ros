#!/usr/bin/env python

import rospy
import os
import yaml
from cv.msg import Object
from cv.srv import ToggleModel
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from detecto.core import Model


class Detector:

    NODE_NAME = 'cv'

    # Load in models and other misc. setup work
    def __init__(self):
        self.bridge = CvBridge()

        # Load in model configurations
        curr_directory = os.path.dirname(__file__)
        with open(os.path.join(curr_directory, '../models/models.yaml')) as f:
            self.models = yaml.load(f)

        # Camera feed topic with default for testing purposes
        self.camera_feed_topic = rospy.get_param('~/cv/camera', '/test_images/image')
        # Toggle model service with camera included at the end
        self.toggle_model_service = 'toggle_model_' + rospy.get_param('~/cv/camera', '')

        # Initialize any enabled models
        for model_name in self.models:
            if self.models[model_name]['enabled']:
                self.init_model(model_name)

    # Initialize model predictor and publisher if not already initialized
    def init_model(self, model_name):
        model = self.models[model_name]

        # Model already initialized; return from method
        if model['predictor'] is not None:
            return

        path = os.path.dirname(__file__)
        weights_file = os.path.join(path, '../models', model['weights'])

        predictor = Model.load(weights_file, model['classes'])
        publisher = rospy.Publisher(model['topic'] + rospy.get_param('~/cv/camera', ''), Object, queue_size=10)

        model['predictor'] = predictor
        model['publisher'] = publisher

    # Camera subscriber callback; publishes predictions for each frame
    def detect(self, img_msg):
        image = self.bridge.imgmsg_to_cv2(img_msg, 'rgb8')

        for model_name in self.models:
            model = self.models[model_name]

            # Generate predictions for each enabled model
            if model['enabled']:
                # Initialize predictor if not already
                self.init_model(model_name)

                preds = model['predictor'].predict_top(image)
                self.publish_predictions(preds, model['publisher'])

    # Publish predictions with the given publisher
    def publish_predictions(self, preds, publisher):
        if not len(preds[0]): # if there are no predictions
            publisher.publish(None)
        else:
            for label, box, score in zip(*preds):
                object_msg = Object()

                object_msg.label = label
                object_msg.score = score

                object_msg.xmin = box[0].item()
                object_msg.ymin = box[1].item()
                object_msg.xmax = box[2].item()
                object_msg.ymax = box[3].item()

                # Safety check that publisher is not None
                if publisher:
                    publisher.publish(object_msg)

    # Service for toggling specific models on and off
    def toggle_model(self, req):
        if req.model_name in self.models:
            model = self.models[req.model_name]
            model['enabled'] = req.enabled

            # Delete model from memory if setting to disabled
            if not model['enabled']:
                model['predictor'] = None
                model['publisher'] = None

            return True

        return False

    # Initialize node and set up Subscriber to generate and
    # publish predictions at every camera frame
    def run(self):
        rospy.init_node(self.NODE_NAME)
        rospy.Subscriber(self.camera_feed_topic, Image, self.detect)

        # Allow service for toggling of models
        rospy.Service(self.toggle_model_service, ToggleModel, self.toggle_model)

        # Keep node running until shut down
        rospy.spin()


if __name__ == '__main__':
    Detector().run()
