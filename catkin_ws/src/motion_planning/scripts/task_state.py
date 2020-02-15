import rospy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist

class TaskState:

    STATE_TOPIC = 'state'
    DESIRED_POSE_TOPIC = 'controls/desired_pose_global'
    DESIRED_TWIST_POWER_TOPIC = 'controls/desired_twist_power'

    def __init__(self):
        self.state_listener = rospy.Subscriber(self.STATE_TOPIC, Odometry, self._on_receive_state)
        self.desired_pose_global_publisher = rospy.Publisher(self.DESIRED_POSE_TOPIC, Pose, queue_size=5)
        self.desired_twist_power_publisher = rospy.Publisher(self.DESIRED_TWIST_POWER_TOPIC, Twist, queue_size=5)
        self.state = None

    def _on_receive_state(self, state):
        """Receive the state, update initial_state if it is empty
        and the task is running"""
        self.state = state