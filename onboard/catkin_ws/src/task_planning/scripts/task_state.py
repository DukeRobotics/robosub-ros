import rospy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Vector3


class TaskState:
    STATE_TOPIC = 'state'
    DESIRED_POSE_TOPIC = 'controls/desired_pose'
    DESIRED_TWIST_POWER_TOPIC = 'controls/desired_power'
    DESIRED_TWIST_VELOCITY_TOPIC = 'controls/desired_twist'
    CV_GATE_DATA_TOPIC = 'cv/gate_data'

    def __init__(self):
        self.state_listener = rospy.Subscriber(self.STATE_TOPIC, Odometry, self._on_receive_state)
        self.desired_pose_global_publisher = rospy.Publisher(self.DESIRED_POSE_TOPIC, Pose, queue_size=5)
        self.desired_twist_power_publisher = rospy.Publisher(self.DESIRED_TWIST_POWER_TOPIC, Twist, queue_size=5)
        self.desired_twist_velocity_publisher = rospy.Publisher(self.DESIRED_TWIST_VELOCITY_TOPIC, Twist, queue_size=5)
        self.state = None
        self.cv_gate_data_listener = rospy.Subscriber(self.CV_GATE_DATA_TOPIC, Vector3, self._on_receive_gate_data)
        self.gate_data = None

    def _on_receive_state(self, state):
        """Receive the state, update initial_state if it is empty
        and the task is running"""
        self.state = state

    def _on_receive_gate_data(self, gate_data):
        self.gate_data = gate_data
