import rospy
import yaml
import resource_retriever as rr
from custom_msgs.msg import CVObject, RectInfo
from geometry_msgs.msg import Pose, Polygon
from std_msgs.msg import Float64
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

        rospy.Subscriber("/cv/bottom/rect", RectInfo, self._on_receive_rect_info)
        self.rect_heights = []

        self.rect_angle_publisher = rospy.Publisher("/task_planning/cv/bottom/rect_angle", Float64, queue_size=1)

        rospy.Subscriber("/cv/front_usb/bounding_box", Polygon, self._on_receive_buoy_bounding_box)

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

    def _on_receive_buoy_bounding_box(self, bounding_box: Polygon) -> None:
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
        dist_x, dist_y = geometry_utils.compute_center_distance(bounding_box, 640, 480)

        # Compute distance between center of bounding box and center of image in meters
        dist_x_meters = dist_x * meters_per_pixel
        dist_y_meters = dist_y * meters_per_pixel

        self.cv_data["buoy_center_distance"] = (dist_x_meters, dist_y_meters)

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