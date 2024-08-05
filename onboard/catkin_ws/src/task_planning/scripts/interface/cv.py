import rospy
import yaml
import resource_retriever as rr
from custom_msgs.msg import CVObject, RectInfo
from geometry_msgs.msg import Pose, Polygon
from std_msgs.msg import Float64
from vision_msgs.msg import Detection2DArray
from vision_msgs.msg import Detection2D
from vision_msgs.msg import ObjectHypothesisWithPose
from utils.other_utils import singleton
from utils import geometry_utils


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
    # TODO We may want a better way to sync this between here and the cv node
    CV_MODEL = "yolov7_tiny_2023_main"

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

        rospy.Subscriber("/cv/bottom/rect_angle", Float64, self._on_receive_rect_angle)
        self.rect_angles = []

        rospy.Subscriber("/cv/bottom/rect_dist", Float64, self._on_receive_rect_dist)
        self.rect_dists = []

        rospy.Subscriber("/cv/bottom/x", RectInfo, self._on_receive_rect_info)
        self.rect_heights = []

        self.rect_angle_publisher = rospy.Publisher("/task_planning/cv/bottom/rect_angle", Float64, queue_size=1)

        rospy.Subscriber("/cv/front_usb/bounding_box", CVObject, self._on_receive_buoy_bounding_box)

        # rospy.Subscriber('/yolov7/detection', Detection2DArray, self._on_receive_gate_detection)

        rospy.Subscriber('/cv/front/gate_red_cw', CVObject, self._on_receive_gate_red_cw_detection_depthai)
        rospy.Subscriber('/cv/front/gate_whole', CVObject, self._on_receive_gate_whole_detection_depthai)

    def _on_receive_cv_data(self, cv_data: CVObject, object_type: str) -> None:
        """
        Parse the received CV data and store it

        Args:
            cv_data: The received CV data as a CVObject
            object_type: The name/type of the object
        """
        self.cv_data[object_type] = cv_data

    # TODO add useful methods for getting data

    def _on_receive_rect_angle(self, angle: Float64) -> None:
        """
        Parse the received angle of the blue rectangle and store it

        Args:
            angle: The received angle of the blue rectangle in degrees
        """
        filter_len = 10
        skip = 0
        if len(self.rect_angles) == filter_len:
            self.rect_angles.pop(0)

        self.rect_angles.append(angle.data)

        self.cv_data["blue_rectangle_angle"] = sum(self.rect_angles[skip:filter_len-skip]) / len(self.rect_angles)
        self.rect_angle_publisher.publish(self.cv_data["blue_rectangle_angle"])

    def _on_receive_rect_dist(self, dist: Float64) -> None:
        """
        Parse the received dist of the blue rectangle and store it

        Args:
            dist: The received dist of the blue rectangle in pixels
        """
        filter_len = 10
        skip = 0
        if len(self.rect_dists) == filter_len:
            self.rect_dists.pop(0)

        self.rect_dists.append(dist.data)

        self.cv_data["blue_rectangle_dist"] = sum(self.rect_dists[skip:filter_len-skip]) / len(self.rect_dists)

    def _on_receive_rect_info(self, rect_info: RectInfo) -> None:
        """
        Parse the received info of the blue rectangle and store it

        Args:
            rect_info: The received info of the blue rectangle
        """
        filter_len = 10
        skip = 0
        if len(self.rect_heights) == filter_len:
            self.rect_heights.pop(0)

        self.rect_heights.append(rect_info.height)

        self.cv_data["blue_rectangle_height"] = sum(self.rect_heights[skip:filter_len-skip]) / len(self.rect_heights)

        # Based on rect_info.center_y and height, determine if rect is touching top and/or bottom of frame
        self.cv_data["blue_rectangle_touching_top"] = rect_info.center_y - rect_info.height / 2 <= 0
        self.cv_data["blue_rectangle_touching_bottom"] = rect_info.center_y + rect_info.height / 2 >= 480

    def _on_receive_buoy_bounding_box(self, bounding_box: CVObject) -> None:
        """
        Parse the received bounding box of the buoy and store it

        Args:
            bounding_box: The received bounding box of the buoy
        """
        self.cv_data["buoy_bounding_box"] = bounding_box

        # Compute width of bounding box
        width, height = geometry_utils.compute_bbox_dimensions(bounding_box)

        self.cv_data["buoy_dimensions"] = (width, height)

        # Get meters per pixel
        meters_per_pixel = self.BUOY_WIDTH / width

        self.cv_data["buoy_meters_per_pixel"] = meters_per_pixel

        # Compute distance between center of bounding box and center of image
        # Here, image x is robot's y, and image y is robot's z
        dist_x, dist_y = geometry_utils.compute_center_distance(bounding_box, *self.MONO_CAM_IMG_SHAPE)

        # Compute distance between center of bounding box and center of image in meters
        dist_x_meters = dist_x * meters_per_pixel
        dist_y_meters = dist_y * meters_per_pixel

        self.cv_data["buoy"] = {
            "x": self.mono_cam_dist_with_obj_width(width, self.BUOY_WIDTH),
            "y": dist_x_meters,
            "z": dist_y_meters,
            "secs": bounding_box.header.stamp.secs
        }

        # rospy.loginfo("Buoy properties: %s", self.cv_data["buoy_properties"])

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

        dist_with_obj_width = self.mono_cam_dist_with_obj_width(bbox_width, self.GATE_IMAGE_WIDTH)
        # dist_with_obj_height = self.mono_cam_dist_with_obj_height(bbox_height, self.GATE_IMAGE_HEIGHT)
        # dist_to_obj = (dist_with_obj_width + dist_with_obj_height) / 2
        dist_to_obj = dist_with_obj_width

        self.cv_data[gate_class + "_properties"] = {
            "bbox_width": bbox_width,
            "bbox_height": bbox_height,
            "meters_per_pixel": meters_per_pixel,
            "x": dist_to_obj,
            "y": dist_x_meters,
            "z": dist_y_meters,
        }

        # log_dict = {
        #     "x": dist_to_obj,
        #     "y": dist_x_meters,
        #     "z": dist_y_meters,
        # }

        # if gate_class == "gate_red_cw":
        #     rospy.loginfo(log_dict)

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