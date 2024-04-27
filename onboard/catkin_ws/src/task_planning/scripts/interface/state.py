import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from robot_localization.srv import SetPose
from tf2_ros.buffer import Buffer
from utils.other_utils import singleton


@singleton
class State:
    """
    Interface for the state of the robot.

    Attributes:
        _instance: The singleton instance of this class. Is a static attribute.
        state: The current state of the robot.
        tfBuffer: The transform buffer for the robot.
        _reset_pose: The service proxy for resetting the pose
    """

    # ROS topics for the state and resetting the pose
    STATE_TOPIC = 'state'
    RESET_POSE_SERVICE = '/set_pose'

    def __init__(self, bypass: bool = False, tfBuffer: Buffer = None):
        """
        Args:
            tfBuffer: The transform buffer for the robot
        """
        self.bypass = bypass
        if tfBuffer:
            self._tfBuffer = tfBuffer

        rospy.Subscriber(self.STATE_TOPIC, Odometry, self._on_receive_state)
        self._state = None
        self._init_state = None

        if not bypass:
            rospy.wait_for_service(self.RESET_POSE_SERVICE)
        self._reset_pose = rospy.ServiceProxy(self.RESET_POSE_SERVICE, SetPose)

    @property
    def state(self) -> Odometry:
        """
        The state
        """
        return self._state

    @property
    def init_state(self) -> Odometry:
        """
        The first state recieved
        """
        return self._init_state

    @property
    def tfBuffer(self) -> Buffer:
        """
        The transform buffer
        """
        return self._tfBuffer

    def _on_receive_state(self, state):
        if not self._init_state:
            self._init_state = state
        self._state = state

    def reset_pose(self):
        """
        Reset the pose
        """
        poseCov = PoseWithCovarianceStamped()
        poseCov.pose.pose.orientation.w = 1
        if not self.bypass:
            self._reset_pose(poseCov)
