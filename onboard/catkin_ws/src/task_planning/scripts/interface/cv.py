import rospy
from custom_msgs.msg import CVObject


class CVInterface:
    REL_PATH = "onboard/catkin_ws/src/cv/models/depthai_models.yaml"  # TODO: read from fil
    CV_DATA_TOPICS = ["/cv/front/bin/left",
                      "/cv/front/bootleggerbuoy/left",
                      "/cv/front/gate/left",
                      "/cv/front/gateleftchild/left",
                      "/cv/front/gaterightchild/left",
                      "/cv/front/gmanbuoy/left",
                      "/cv/front/octagon/left",
                      "/cv/front/pole/left",
                      "/cv/front/straightpathmarker/left"]

    def __init__(self):
        self.cv_data = {}
        # TODO: make this work
        '''
        model = self.models[model_name]
        publisher_dict = {}
        for model_class in model['classes']:
            publisher_name = f"cv/{self.camera}/{model_class}"
            publisher_dict[model_class] = rospy.Publisher(publisher_name,
                                                          CVObject,
                                                          queue_size=10)
        '''
        for topic in self.CV_DATA_TOPICS:
            name = topic.replace("/left", "").split("/")[-1]
            self.cv_data[name] = None
            rospy.Subscriber(topic, CVObject, self._on_receive_cv_data, name)

    def _on_receive_cv_data(self, cv_data, object_type):
        self.cv_data[object_type] = cv_data

    # TODO add useful methods for getting data
    def get_data(self, name):
        return self.cv_data[name]
