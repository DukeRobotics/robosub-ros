import rospy

from custom_msgs.msg import TopicNames
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist
from custom_msgs.msg import CVObject


class TaskState:
    STATE_TOPIC = TopicNames.state
    DESIRED_POSE_TOPIC = TopicNames.controls_desired_pose
    DESIRED_TWIST_POWER_TOPIC = TopicNames.controls_desired_power
    DESIRED_TWIST_VELOCITY_TOPIC = TopicNames.controls_desired_twist
    CV_GATE_DATA_TOPIC = TopicNames.cv_start_gate_left
    CV_GATE_TICK_DATA_TOPIC = TopicNames.cv_start_tick_left

    def __init__(self):
        self.state_listener = rospy.Subscriber(self.STATE_TOPIC, Odometry, self._on_receive_state)
        self.desired_pose_global_publisher = rospy.Publisher(self.DESIRED_POSE_TOPIC, Pose, queue_size=5)
        self.desired_twist_power_publisher = rospy.Publisher(self.DESIRED_TWIST_POWER_TOPIC, Twist, queue_size=5)
        self.desired_twist_velocity_publisher = rospy.Publisher(self.DESIRED_TWIST_VELOCITY_TOPIC, Twist, queue_size=5)
        self.state = None

        self.cv_data = {
            'gate': None,
            'gate_tick': None
        }
        self.cv_gate_data_listener = rospy.Subscriber(
            self.CV_GATE_DATA_TOPIC, CVObject, self._on_receive_gate_data, 'gate')
        self.cv_gate_tick_data_listener = rospy.Subscriber(self.CV_GATE_TICK_DATA_TOPIC, CVObject,
                                                           self._on_receive_gate_data, 'gate_tick')

    def _on_receive_state(self, state):
        """Receive the state, update initial_state if it is empty
        and the task is running"""
        self.state = state

    def _on_receive_gate_data(self, gate_data, object_type):
        self.cv_data[object_type] = gate_data
