import rospy
from custom_msgs.msg import CVObject


class CVInterface:
    REL_PATH = "onboard/catkin_ws/src/cv/models/depthai_models.yaml"  # TODO: read from fil
    CV_DATA_TOPICS = ["/cv/front/gate",
                      "/cv/front/gate_side",
                      "/cv/front/gate_tick",
                      "/cv/front/gate_top",
                      "/cv/front/start_gate",
                      "/cv/front/buoy_bootlegger",
                      "/cv/front/buoy_gman"]

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
            name = topic.split("/")[-1]
            self.cv_data[name] = None
            rospy.Subscriber(topic, CVObject, self._on_receive_cv_data, name)

    def _on_receive_cv_data(self, cv_data, object_type):
        self.cv_data[object_type] = cv_data

    # TODO add useful methods for getting data
    def get_data(self, name):
        return self.cv_data[name]
