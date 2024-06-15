import rospy
import yaml
import resource_retriever as rr
from custom_msgs.msg import CVObject
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
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
    # TODO We may want a better way to sync this between here and the cv node
    CV_MODEL = "yolov7_tiny_2023_main"

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
        if len(self.rect_angles) == filter_len:
            self.rect_angles.pop(0)

        self.rect_angles.append(angle.data)

        self.cv_data["blue_rectangle_angle"] = sum(self.rect_angles) / len(self.rect_angles)

    def _on_receive_rect_dist(self, dist: Float64) -> None:
        """
        Parse the received dist of the blue rectangle and store it

        Args:
            dist: The received dist of the blue rectangle in pixels
        """
        filter_len = 10
        if len(self.rect_dists) == filter_len:
            self.rect_dists.pop(0)

        self.rect_dists.append(dist.data)

        self.cv_data["blue_rectangle_dist"] = sum(self.rect_dists) / len(self.rect_dists)

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