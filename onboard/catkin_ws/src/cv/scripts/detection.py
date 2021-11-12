#!/usr/bin/env python3

import rospy
import yaml
from stereo import StereoDetector
from custom_msgs.msg import CVObject
from custom_msgs.srv import EnableModel
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from detecto.core import Model
import resource_retriever as rr


class Detector:

    # Load in models and other misc. setup work
    def __init__(self):
        rospy.init_node('cv', anonymous=True)

        self.bridge = CvBridge()
        self.camera = rospy.get_param('~camera')

        # Load in model configurations
        with open(rr.get_filename('package://cv/models/models.yaml', use_protocol=False)) as f:
            self.models = yaml.safe_load(f)

        self.stereo_detector = None
        if self.camera == 'stereo':
            self.camera_feed_topic_left = '/camera/left/image_raw'
            self.camera_feed_topic_right = '/camera/right/image_raw'
            self.img_left = None
            self.img_right = None
            self.stereo_detector = StereoDetector()
        else:
            # The topic that the camera publishes its feed to
            self.camera_feed_topic = '/camera/{}/image_raw'.format(self.camera)

        # Toggle model service name
        self.enable_service = f'enable_model_{self.camera}'

    # Initialize model predictor and publisher if not already initialized
    def init_model(self, model_name):
        model = self.models[model_name]

        # Model already initialized; return from method
        if model.get('predictor') is not None:
            return

        weights_file = rr.get_filename(f"package://cv/models/{model['weights']}", use_protocol=False)

        predictor = Model.load(weights_file, model['classes'])

        publisher_name = f"{model['topic']}/{self.camera}"
        publisher = rospy.Publisher(publisher_name, CVObject, queue_size=10)

        model['predictor'] = predictor
        model['publisher'] = publisher

    def stereo_left(self, img_msg):
        self.img_left = img_msg
        if self.img_right is not None:
            self.stereo()
            self.stereo_cleanup()

    def stereo_right(self, img_msg):
        self.img_right = img_msg
        if self.img_left is not None:
            self.stereo()
            self.stereo_cleanup()

    def stereo(self):
        self.stereo_detector.compute_disparity(self.img_left, self.img_right)
        self.detect(self.img_left)

    def stereo_cleanup(self):
        self.img_left = None
        self.img_right = None

    # Camera subscriber callback; publishes predictions for each frame
    def detect(self, img_msg):
        image = self.bridge.imgmsg_to_cv2(img_msg, 'rgb8')

        for model_name in self.models:
            model = self.models[model_name]

            # Generate predictions for each enabled model
            if model.get('enabled'):
                # Initialize predictor if not already
                self.init_model(model_name)

                preds = model['predictor'].predict_top(image)
                self.publish_predictions(preds, model['publisher'], image.shape)

    # Publish predictions with the given publisher
    def publish_predictions(self, preds, publisher, shape):
        labels, boxes, scores = preds

        # If there are no predictions, publish 'none' as the object label
        if not labels:
            object_msg = CVObject()
            object_msg.label = 'none'
            publisher.publish(object_msg)
        else:
            for label, box, score in zip(labels, boxes, scores):
                object_msg = CVObject()
                object_msg.stereo_enabled = False

                object_msg.label = label
                object_msg.score = score

                object_msg.xmin = box[0].item() / shape[1]
                object_msg.ymin = box[1].item() / shape[0]
                object_msg.xmax = box[2].item() / shape[1]
                object_msg.ymax = box[3].item() / shape[0]

                object_msg.height = shape[0]
                object_msg.width = shape[1]

                if self.stereo_detector is not None:
                    object_msg.stereo_enabled = True
                    self.stereo_detector.populate_stereo_info(object_msg, box)

                # Safety check that publisher is not None
                if publisher:
                    publisher.publish(object_msg)

    # Service for toggling specific models on and off
    def enable_model(self, req):
        if req.model_name in self.models:
            model = self.models[req.model_name]
            model['enabled'] = req.enabled

            # Delete model from memory if setting to disabled
            if not model.get('enabled'):
                model['predictor'] = None
                model['publisher'] = None

            return True

        return False

    # Initialize node and set up Subscriber to generate and
    # publish predictions at every camera frame
    def run(self):
        if self.camera == 'stereo':
            rospy.Subscriber(self.camera_feed_topic_left, Image, self.stereo_left)
            rospy.Subscriber(self.camera_feed_topic_right, Image, self.stereo_right)
        else:
            rospy.Subscriber(self.camera_feed_topic, Image, self.detect)

        # Allow service for toggling of models
        rospy.Service(self.enable_service, EnableModel, self.enable_model)

        # Keep node running until shut down
        rospy.spin()


if __name__ == '__main__':
    Detector().run()
