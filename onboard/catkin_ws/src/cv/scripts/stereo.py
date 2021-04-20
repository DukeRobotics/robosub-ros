import cv2
from cv_bridge import CvBridge


class StereoDetector:

    def __init__(self):
        self.bridge = CvBridge()
        # todo initialize constants
        self.disparity = None

    def compute_disparity(self, img_left_raw, img_right_raw):
        img_left = self.bridge.imgmsg_to_cv2(img_left_raw, 'rgb8')
        img_right = self.bridge.imgmsg_to_cv2(img_right_raw, 'rgb8')

        self.disparity = None  # todo

    def cleanup(self):
        self.disparity = None

    # Takes in a CVObject and populates with depth, angle_horiz, and angle_vert
    def populate_stereo_info(self, cv_object):
        pass
