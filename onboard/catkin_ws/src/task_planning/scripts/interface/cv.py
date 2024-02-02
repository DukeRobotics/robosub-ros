import rospy
import yaml
import resource_retriever as rr
from custom_msgs.msg import CVObject
from geometry_msgs.msg import Pose


class CVInterface:
    MODELS_PATH = "package://cv/models/depthai_models.yaml"
    CV_CAMERA = "front"
    # TODO We may want a better way to sync this between here and the cv node
    CV_MODEL = "yolov7_tiny_2023_main"

    def __init__(self):
        self.cv_data = {}

        with open(rr.get_filename(self.MODELS_PATH, use_protocol=False)) as f:
            model = yaml.safe_load(f)[self.CV_MODEL]

            for model_class in model['classes']:
                self.cv_data[model_class] = None
                topic = f"{model['topic']}{self.CV_CAMERA}/{model_class}"
                rospy.Subscriber(topic, CVObject, self._on_receive_cv_data, model_class)

    def _on_receive_cv_data(self, cv_data, object_type):
        self.cv_data[object_type] = cv_data

    # TODO add useful methods for getting data
    def get_data(self, name):
        return self.cv_data[name]

    def get_pose(self, name):
        data = self.get_data(name)
        pose = Pose()
        pose.position.x = data.coords.x
        pose.position.y = data.coords.y
        pose.position.z = data.coords.z
        pose.orientation.x = 0
        pose.orientation.y = 0
        pose.orientation.z = 0
        pose.orientation.w = 1
        return pose
