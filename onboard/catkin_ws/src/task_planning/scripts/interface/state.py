import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from robot_localization.srv import SetPose
from tf2_ros.buffer import Buffer


class State:
    """
    Interface for the state of the robot.

    Attributes:
        _instance: The singleton instance of this class. Is a static attribute.
        state: The current state of the robot.
        tfBuffer: The transform buffer for the robot.
        _reset_pose: The service proxy for resetting the pose
    """

    _instance = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(State, cls).__new__(cls)
            cls._instance.__init__()
        return cls._instance

    # ROS topics for the state and resetting the pose
    STATE_TOPIC = 'state'
    RESET_POSE_SERVICE = '/set_pose'

    def __init__(self, tfBuffer: Buffer):
        """
        Args:
            tfBuffer: The transform buffer for the robot
        """
        self._tfBuffer = tfBuffer

        rospy.Subscriber(self.STATE_TOPIC, Odometry, self._on_receive_state)
        self._state = None

        rospy.wait_for_service(self.RESET_POSE_SERVICE)
        self._reset_pose = rospy.ServiceProxy(self.RESET_POSE_SERVICE, SetPose)

    @property
    def state(self):
        """
        The state
        """
        return self._state

    @property
    def tfBuffer(self):
        """
        The transform buffer
        """
        return self._tfBuffer

    def _on_receive_state(self, state):
        self._state = state

    def reset_pose(self):
        """
        Reset the pose
        """
        poseCov = PoseWithCovarianceStamped()
        poseCov.pose.pose.orientation.w = 1
        self._reset_pose(poseCov)
