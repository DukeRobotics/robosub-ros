import rospy
import yaml
import numpy as np
import resource_retriever as rr
from custom_msgs.msg import CVObject, RectInfo
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import Float64
from vision_msgs.msg import Detection2DArray
from utils.other_utils import singleton


@singleton
class CV:
    """
    Interface for the CV.

    Attributes:
        _instance: The singleton instance of this class. Is a static attribute.
        cv_data: Dictionary of the data of the objects
    """

    MODELS_PATH = "package://cv/models/depthai_models.yaml"
    CV_CAMERA = "front"
    # TODO: We may want a better way to sync this between here and the cv node
    CV_MODEL = "yolov7_tiny_2023_main"

    FRAME_WIDTH = 640
    FRAME_HEIGHT = 320

    BIN_WIDTH = 0.3048  # Width of single bin in meters
    BUOY_WIDTH = 0.2032  # Width of buoy in meters
    GATE_IMAGE_WIDTH = 0.2452  # Width of gate images in meters
    GATE_IMAGE_HEIGHT = 0.2921  # Height of gate images in meters

    MONO_CAM_IMG_SHAPE = (640, 480)  # Width, height in pixels
    MONO_CAM_SENSOR_SIZE = (3.054, 1.718)  # Width, height in mm
    MONO_CAM_FOCAL_LENGTH = 2.65  # Focal length in mm

    def __init__(self, bypass: bool = False):
        self.cv_data = {}
        self.bypass = bypass

        with open(rr.get_filename(self.MODELS_PATH, use_protocol=False)) as f:
            model = yaml.safe_load(f)[self.CV_MODEL]

            for model_class in model['classes']:
                self.cv_data[model_class] = None
                topic = f"{model['topic']}{self.CV_CAMERA}/{model_class}"
                rospy.Subscriber(topic, CVObject, self._on_receive_cv_data, model_class)

        rospy.Subscriber("/cv/bottom/lane_marker_angle", Float64, self._on_receive_lane_marker_angle)
        self.lane_marker_angles = []

        rospy.Subscriber("/cv/bottom/lane_marker_dist", Float64, self._on_receive_lane_marker_dist)
        self.lane_marker_dists = []

        rospy.Subscriber("/cv/bottom/lane_marker", RectInfo, self._on_receive_lane_marker_info)
        self.lane_marker_heights = []

        self.lane_marker_angle_publisher = rospy.Publisher("/task_planning/cv/bottom/lane_marker_angle",
                                                           Float64, queue_size=1)

        rospy.Subscriber("/cv/front_usb/buoy/bounding_box", CVObject, self._on_receive_cv_data, "buoy")

        rospy.Subscriber('/cv/front/gate_red_cw', CVObject, self._on_receive_cv_data, "gate_red_cw")
        rospy.Subscriber('/cv/front/gate_whole', CVObject, self._on_receive_cv_data, "gate_whole")

        rospy.Subscriber("/cv/bottom/bin_blue/bounding_box", CVObject, self._on_receive_cv_data, "bin_blue")
        rospy.Subscriber("/cv/bottom/bin_red/bounding_box", CVObject, self._on_receive_cv_data, "bin_red")
        rospy.Subscriber("/cv/bottom/bin_center/bounding_box", CVObject, self._on_receive_cv_data, "bin_center")

        rospy.Subscriber("/cv/bottom/bin_blue/distance", Point, self._on_receive_distance_data, "bin_blue")
        rospy.Subscriber("/cv/bottom/bin_red/distance", Point, self._on_receive_distance_data, "bin_red")
        rospy.Subscriber("/cv/bottom/bin_center/distance", Point, self._on_receive_distance_data, "bin_center")

        self.bin_distances = {object_type: {"x": [], "y": []} for object_type in ["bin_red", "bin_blue"]}

        rospy.Subscriber("/cv/bottom/path_marker/bounding_box", CVObject, self._on_receive_cv_data, "path_marker")
        rospy.Subscriber("/cv/bottom/path_marker/distance", Point, self._on_receive_distance_data, "path_marker")

        rospy.Subscriber("/cv/front/pink_bins/bounding_box", CVObject, self._on_receive_cv_data, "bin_pink_front")
        rospy.Subscriber("/cv/bottom/pink_bins/bounding_box", CVObject, self._on_receive_cv_data, "bin_pink_bottom")

    def _on_receive_cv_data(self, cv_data: CVObject, object_type: str) -> None:
        """
        Parse the received CV data and store it

        Args:
            cv_data: The received CV data as a CVObject
            object_type: The name/type of the object
        """
        self.cv_data[object_type] = cv_data

    def _on_receive_distance_data(self, distance_data: Point, object_type: str, filter_len=10) -> None:
        """
        Parse the received distance data and store it

        Args:
            distance_data: The received distance data as a Point
            object_type: The name/type of the object
        """
        # TODO: migrate all self.{object}_distances type objects into a single self.distances dictionary
        # TODO: implement a generic moving average filter
        # TODO: integrate _on_receive_lane_marker_dist
        if object_type == "path_marker":
            self.cv_data["path_marker_distance"] = distance_data
            return

        if len(self.bin_distances[object_type]["x"]) == filter_len:
            self.bin_distances[object_type]["x"].pop(0)
        if len(self.bin_distances[object_type]["y"]) == filter_len:
            self.bin_distances[object_type]["y"].pop(0)

        self.bin_distances[object_type]["x"].append(distance_data.x)
        self.bin_distances[object_type]["y"].append(distance_data.y)

        if "bin_red_distance" not in self.cv_data:
            self.cv_data["bin_red_distance"] = Point()
        if "bin_blue_distance" not in self.cv_data:
            self.cv_data["bin_blue_distance"] = Point()

        distance_x = sum(self.bin_distances[object_type]["x"]) / len(self.bin_distances[object_type]["x"])
        self.cv_data[f"{object_type}_distance"].x = distance_x

        distance_y = sum(self.bin_distances[object_type]["y"]) / len(self.bin_distances[object_type]["y"])
        self.cv_data[f"{object_type}_distance"].y = distance_y

        red_data = self.cv_data["bin_red_distance"]
        blue_data = self.cv_data["bin_blue_distance"]

        if (red_data := self.cv_data["bin_red_distance"]) and (blue_data := self.cv_data["bin_blue_distance"]):
            red_x, red_y = red_data.x, red_data.y
            blue_x, blue_y = blue_data.x, blue_data.y

            center_red_x = self.FRAME_WIDTH / 2 - red_x
            center_red_y = self.FRAME_HEIGHT / 2 - red_y
            center_blue_x = self.FRAME_WIDTH / 2 - blue_x
            center_blue_y = self.FRAME_HEIGHT / 2 - blue_y

            # TODO: do some mathy stuff to make this cleaner
            angle = np.arctan2(center_red_y - center_blue_y, center_red_x - center_blue_x)
            if angle > np.pi:
                angle -= 2 * np.pi
            elif angle < -np.pi:
                angle += 2 * np.pi

            if angle > np.pi / 2:
                angle -= np.pi
            elif angle < -np.pi / 2:
                angle += np.pi

            self.cv_data["bin_angle"] = angle

    def _on_receive_lane_marker_angle(self, angle: Float64) -> None:
        """
        Parse the received angle of the blue rectangle and store it

        Args:
            angle: The received angle of the blue rectangle in degrees
        """
        filter_len = 10
        skip = 0
        if len(self.lane_marker_angles) == filter_len:
            self.lane_marker_angles.pop(0)

        self.lane_marker_angles.append(angle.data)

        lane_marker_angle = sum(self.lane_marker_angles[skip:filter_len-skip]) / len(self.lane_marker_angles)
        self.cv_data["lane_marker_angle"] = lane_marker_angle
        self.lane_marker_angle_publisher.publish(self.cv_data["lane_marker_angle"])

    # TODO: remove this and integrate into _on_receive_distance_data
    def _on_receive_lane_marker_dist(self, dist: Float64) -> None:
        """
        Parse the received dist of the blue rectangle and store it

        Args:
            dist: The received dist of the blue rectangle in pixels
        """
        filter_len = 10
        skip = 0
        if len(self.lane_marker_dists) == filter_len:
            self.lane_marker_dists.pop(0)

        self.lane_marker_dists.append(dist.data)

        lane_marker_dist = sum(self.lane_marker_dists[skip:filter_len-skip]) / len(self.lane_marker_dists)
        self.cv_data["lane_marker_dist"] = lane_marker_dist

    def _on_receive_lane_marker_info(self, lane_marker_info: RectInfo) -> None:
        """
        Parse the received info of the blue rectangle and store it

        Args:
            rect_info: The received info of the blue rectangle
        """
        filter_len = 10
        skip = 0
        if len(self.lane_marker_heights) == filter_len:
            self.lane_marker_heights.pop(0)

        self.lane_marker_heights.append(lane_marker_info.height)

        lane_marker_height = sum(self.lane_marker_heights[skip:filter_len-skip]) / len(self.lane_marker_heights)
        self.cv_data["lane_marker_height"] = lane_marker_height

        # Based on lane_marker_info.center_y and height, determine if lane marker is touching top and/or bottom of frame
        self.cv_data["lane_marker_touching_top"] = lane_marker_info.center_y - lane_marker_info.height / 2 <= 0
        self.cv_data["lane_marker_touching_bottom"] = lane_marker_info.center_y + lane_marker_info.height / 2 >= 480

    def _on_receive_gate_red_cw_detection_depthai(self, msg: CVObject) -> None:
        """
        Parse the received detection of the red gate and store it

        Args:
            msg: The received detection of the red gate
        """
        self.cv_data["gate_red_cw_properties"] = {
            "x": msg.coords.x,
            "y": msg.coords.y,
            "z": msg.coords.z
        }

    def _on_receive_gate_whole_detection_depthai(self, msg: CVObject) -> None:
        """
        Parse the received detection of the whole gate and store it

        Args:
            msg: The received detection of the whole gate
        """
        self.cv_data["gate_whole_properties"] = {
            "x": msg.coords.x,
            "y": msg.coords.y,
            "z": msg.coords.z,
            "yaw": msg.coords.yaw,
            "secs": msg.header.stamp.secs
        }

    def _on_receive_gate_detection(self, msg: Detection2DArray) -> None:
        for detection in msg.detections:
            for result in detection.results:
                if result.id == 0:  # gate_blue_ccw
                    self.cv_data["gate_blue_ccw_bbox"] = detection.bbox
                elif result.id == 1:  # gate_red_cw
                    self.cv_data["gate_red_cw_bbox"] = detection.bbox

        highest_confidence_blue = -1
        highest_confidence_red = -1
        best_bbox_blue = None
        best_bbox_red = None

        for detection in msg.detections:
            for result in detection.results:
                if result.id == 0 and result.score > highest_confidence_blue:  # gate_blue_ccw
                    highest_confidence_blue = result.score
                    best_bbox_blue = detection.bbox
                elif result.id == 1 and result.score > highest_confidence_red:  # gate_red_cw
                    highest_confidence_red = result.score
                    best_bbox_red = detection.bbox

        if best_bbox_blue is not None:
            self.cv_data["gate_blue_ccw_bbox"] = best_bbox_blue
            self.compute_gate_properties("gate_blue_ccw")
        if best_bbox_red is not None:
            self.cv_data["gate_red_cw_bbox"] = best_bbox_red
            self.compute_gate_properties("gate_red_cw")

    def compute_gate_properties(self, gate_class):
        if gate_class + "_bbox" not in self.cv_data or self.cv_data[gate_class + "_bbox"] is None:
            rospy.logwarn(f"No bounding box data available for {gate_class}")
            return None

        bbox = self.cv_data[gate_class + "_bbox"]

        # Assuming bbox is of type vision_msgs/BoundingBox2D
        bbox_width, bbox_height = bbox.size_x, bbox.size_y

        # Compute the meters per pixel (assuming GATE_IMAGE_WIDTH is the real width in meters)
        meters_per_pixel = self.GATE_IMAGE_WIDTH / bbox_width

        # Use geometry_utils to compute center distances
        # Here, image x is robot's y, and image y is robot's z
        dist_x = bbox.center.x - self.MONO_CAM_IMG_SHAPE[0] / 2
        dist_y = bbox.center.y - self.MONO_CAM_IMG_SHAPE[1] / 2

        # Compute distance between center of bounding box and center of image in meters
        dist_x_meters = dist_x * meters_per_pixel * -1
        dist_y_meters = dist_y * meters_per_pixel * -1

        dist_to_obj = self.mono_cam_dist_with_obj_width(bbox_width, self.GATE_IMAGE_WIDTH)

        self.cv_data[gate_class + "_properties"] = {
            "bbox_width": bbox_width,
            "bbox_height": bbox_height,
            "meters_per_pixel": meters_per_pixel,
            "x": dist_to_obj,
            "y": dist_x_meters,
            "z": dist_y_meters,
        }

    def mono_cam_dist_with_obj_width(self, width_pixels, width_meters):
        return (self.MONO_CAM_FOCAL_LENGTH * width_meters * self.MONO_CAM_IMG_SHAPE[0]) \
            / (width_pixels * self.MONO_CAM_SENSOR_SIZE[0])

    def mono_cam_dist_with_obj_height(self, height_pixels, height_meters):
        return (self.MONO_CAM_FOCAL_LENGTH * height_meters * self.MONO_CAM_IMG_SHAPE[1]) \
            / (height_pixels * self.MONO_CAM_SENSOR_SIZE[1])

    def get_pose(self, name: str) -> Pose:
        """
        Get the pose of a detected object

        Args:
            name: The name/type of the object

        Returns:
            The pose of the object
        """
        data = self.cv_data[name]
        pose = Pose()
        pose.position.x = data.coords.x
        pose.position.y = data.coords.y
        pose.position.z = data.coords.z
        pose.orientation.x = 0
        pose.orientation.y = 0
        pose.orientation.z = 0
        pose.orientation.w = 1
        return pose
