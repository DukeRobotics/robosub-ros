import rospy

from custom_msgs.msg import CVObject


class CVInterface:
    CV_DATA_TOPICS = ["/cv/simulation/bin/left",
                      "/cv/simulation/bootleggerbuoy/left",
                      "/cv/simulation/gate/left",
                      "/cv/simulation/gateleftchild/left",
                      "/cv/simulation/gaterightchild/left",
                      "/cv/simulation/gmanbuoy/left",
                      "/cv/simulation/octagon/left",
                      "/cv/simulation/pole/left",
                      "/cv/simulation/straightpathmarker/left"]
    
    def __init__(self):
        self.cv_data = {}
        for topic in self.CV_DATA_TOPICS:
            name = topic.replace("/left", "").split("/")[-1]
            self.cv_data[name] = None
            rospy.Subscriber(topic, CVObject, self._on_receive_cv_data, name)

    def _on_receive_cv_data(self, cv_data, object_type):
        self.cv_data[object_type] = cv_data

    # TODO add useful methods for getting data