import cv2
from cv_bridge import CvBridge
import numpy as np
import resource_retriever as rr


class StereoDetector:

    def __init__(self):
        self.bridge = CvBridge()
        # todo initialize constants
        self.disparity = None
        self.map_3d = None
        self.rmap = np.load(rr.get_filename("package://cv/data/stereocamera_calibration_remapping_matrices", 
                                            use_protocol=False))
        self.img_size = None
        self.fov = np.load(rr.get_filename("package://cv/data/fov_data", use_protocol=False))
        self.Q = np.load(rr.get_filename("package://cv/data/Q_matrix", use_protocol=False))

    # Compute disparity map for given two images
    def compute_disparity(self, img_left_raw, img_right_raw):
        img_left = self.bridge.imgmsg_to_cv2(img_left_raw, 'mono8')
        img_right = self.bridge.imgmsg_to_cv2(img_right_raw, 'mono8')

        self.img_size = img_left.shape

        img_left_remapped = cv2.remap(img_left, self.rmap[0][0], self.rmap[0][1], cv2.INTER_LINEAR)
        img_right_remapped = cv2.remap(img_right, self.rmap[1][0], self.rmap[1][1], cv2.INTER_LINEAR)

        stereo = cv2.StereoSGBM_create(numDisparities=80, blockSize=21)
        disparity = stereo.compute(img_left_remapped, img_right_remapped)
        disparity = (disparity / 16).astype(disparity.dtype)

        self.disparity = disparity
        self.map_3d = cv2.reprojectImageTo3D(self.disparity, self.Q, True) / 1000

    # Takes in a CVObject and populates with depth, distance, angle_horiz, and angle_vert
    def populate_stereo_info(self, cv_object, box):
        horizontal_angle, vertical_angle = self._compute_angles(box)
        depth, distance = self._compute_depth_and_distance(box)

        # Fill in fields in message
        cv_object.depth = depth
        cv_object.distance = distance

        cv_object.angle_horiz = horizontal_angle
        cv_object.angle_vert = vertical_angle

        cv_object.stereo_enabled = 1

    def _compute_angles(self, box):
        x_center = (box[0].item() + box[2].item())/2
        y_center = (box[1].item() + box[3].item())/2

        degrees_per_pixel_horizontal = self.fov[0] / self.img_size[1]
        degrees_per_pixel_vertical = self.fov[1] / self.img_size[0]

        horizontal_angle = (x_center - (self.img_size[1]/2)) * degrees_per_pixel_horizontal
        vertical_angle = -(y_center - (self.img_size[0]/2)) * degrees_per_pixel_vertical

        return horizontal_angle, vertical_angle

    def _compute_depth_and_distance(self, box):
        bbox_img = np.zeros(self.img_size, np.uint8)
        bbox_img = cv2.rectangle(bbox_img, 
                                (int(box[0].item()), int(box[1].item())), 
                                (int(box[2].item()), int(box[3].item())), 
                                1, -1)

        # Assuming the bounding box is predicted on the left image
        bbox_remapped = cv2.remap(bbox_img, self.rmap[0][0], self.rmap[0][1], cv2.INTER_LINEAR)
        mask = cv2.merge([bbox_remapped, bbox_remapped, bbox_remapped])

        bbox_map_3d = self.map_3d * mask  # Apply bounding box mask to 3d map

        depth_map = bbox_map_3d[:, :, 2]

        depth = np.median(depth_map[depth_map > 0])
        xpos = np.median(bbox_map_3d[:, :, 0][depth_map > 0])
        ypos = np.median(bbox_map_3d[:, :, 1][depth_map > 0])
        distance = np.sqrt(xpos ** 2 + ypos ** 2 + depth ** 2)

        return depth, distance
