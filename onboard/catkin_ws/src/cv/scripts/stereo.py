import cv2
from cv_bridge import CvBridge


class StereoDetector:

    def __init__(self, raw_img_left, raw_img_right):
        self.bridge = CvBridge()
        self.img_left = self.bridge.imgmsg_to_cv2(raw_img_left, 'rgb8')
        self.img_right = self.bridge.imgmsg_to_cv2(raw_img_right, 'rgb8')

        self.disparity = self._calculate_disparity(self.img_left, self.img_right)

    def _calculate_disparity(self, img_left, img_right):
        return 0

    # Takes in a CVObject and populates with depth, angle_horiz, and angle_vert
    def populate_stereo_info(self, cv_object):
        pass
