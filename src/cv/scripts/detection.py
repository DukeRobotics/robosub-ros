#!/usr/bin/env python

import rospy
import os
from cv.msg import Object
from cv.srv import ToggleModel
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from detecto.core import Model

import torch  # TODO testing toggle_model


class Detector:

    NODE_NAME = 'cv'
    CAMERA_FEED_TOPIC = '/test_images/image'

    TOGGLE_MODEL_SERVICE = 'toggle_model'

    MODELS = {
        'buoy': {
            'enabled': True,
            'classes': ['alien', 'bat', 'witch', 'brick', '_start_gate', '_start_tick', '_'],
            'weights': '../models/buoy.pth',
            'topic': '/cv/buoy',
            'predictor': None,  # Set dynamically based on what's running to save memory
            'publisher': None,  # Release model if enabled set to false - save memory?
        },
    }

    # Load in models and other misc. setup work
    def __init__(self):
        self.bridge = CvBridge()

        # Initialize any enabled models
        for model_name in self.MODELS:
            if self.MODELS[model_name]['enabled']:
                self.init_model(model_name)

    # Initialize model predictor and publisher if not already initialized
    def init_model(self, model_name):
        model = self.MODELS[model_name]

        # Model already initialized; return from method
        if model['predictor'] is not None:
            return

        path = os.path.dirname(__file__)
        weights_file = os.path.join(path, model['weights'])

        predictor = Model.load(weights_file, model['classes'])
        publisher = rospy.Publisher(model['topic'], Object, queue_size=10)

        model['predictor'] = predictor
        model['publisher'] = publisher

    # Camera subscriber callback; publishes predictions for each frame
    def detect(self, img_msg):
        image = self.bridge.imgmsg_to_cv2(img_msg, 'rgb8')

        for model_name in self.MODELS:
            model = self.MODELS[model_name]

            # Generate predictions for each enabled model
            if model['enabled']:
                # preds = model['predictor'].predict_top(image)  # TODO testing toggle_model
                preds = ([model_name], [torch.ones(4)], [0.5])   # TODO testing toggle_model
                self.publish_predictions(preds, model['publisher'])

    # Publish predictions with the given publisher
    def publish_predictions(self, preds, publisher):
        for label, box, score in zip(*preds):
            object_msg = Object()

            object_msg.label = label
            object_msg.score = score

            object_msg.xmin = box[0].item()
            object_msg.ymin = box[1].item()
            object_msg.xmax = box[2].item()
            object_msg.ymax = box[3].item()

            publisher.publish(object_msg)

    # Service for toggling specific models on and off
    def toggle_model(self, req):
        if req.model_name in self.MODELS:
            model = self.MODELS[req.model_name]
            model['enabled'] = req.enabled

            if model['enabled']:
                self.init_model(req.model_name)

        return True

    # Initialize node and set up Subscriber to generate and
    # publish predictions at every camera frame
    def run(self):
        rospy.init_node(self.NODE_NAME)
        rospy.Subscriber(self.CAMERA_FEED_TOPIC, Image, self.detect)

        # Allow service for toggling of models
        rospy.Service(self.TOGGLE_MODEL_SERVICE, ToggleModel, self.toggle_model)

        # Keep node running until shut down
        rospy.spin()


if __name__ == '__main__':
    Detector().run()
