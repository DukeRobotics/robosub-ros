import rospy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist
from custom_msgs.msg import CVObject


class TaskState:
    STATE_TOPIC = '/state'
    DESIRED_POSE_TOPIC = 'controls/desired_pose'
    DESIRED_TWIST_POWER_TOPIC = 'controls/desired_power'
    DESIRED_TWIST_VELOCITY_TOPIC = 'controls/desired_twist'
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
        self.state_listener = rospy.Subscriber(self.STATE_TOPIC, Odometry, self._on_receive_state)
        self.desired_pose_global_publisher = rospy.Publisher(self.DESIRED_POSE_TOPIC, Pose, queue_size=5)
        self.desired_twist_power_publisher = rospy.Publisher(self.DESIRED_TWIST_POWER_TOPIC, Twist, queue_size=5)
        self.desired_twist_velocity_publisher = rospy.Publisher(self.DESIRED_TWIST_VELOCITY_TOPIC, Twist, queue_size=5)
        self.state = None

        self.cv_data = {}
        for topic in self.CV_DATA_TOPICS:
            name = topic.replace("/left", "").split("/")[-1]
            self.cv_data[name] = None
            rospy.Subscriber(topic, CVObject, self._on_receive_cv_data, name)

    def _on_receive_state(self, state):
        """Receive the state, update initial_state if it is empty
        and the task is running"""
        self.state = state

    def _on_receive_cv_data(self, cv_data, object_type):
        self.cv_data[object_type] = cv_data
