import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from robot_localization.srv import SetPose


class StateInterface:
    """
    Interface for the state of the robot.

    Attributes:
        state: The current state of the robot.
        tfBuffer: The transform buffer for the robot.
    """

    STATE_TOPIC = 'state'
    RESET_POSE_SERVICE = '/set_pose'

    def __init__(self, tfBuffer):
        self._tfBuffer = tfBuffer

        rospy.Subscriber(self.STATE_TOPIC, Odometry, self._on_receive_state)
        self._state = None

        rospy.wait_for_service(self.RESET_POSE_SERVICE)
        self._reset_pose = rospy.ServiceProxy(self.RESET_POSE_SERVICE, SetPose)

    @property
    def state(self):
        return self._state

    @property
    def tfBuffer(self):
        return self._tfBuffer

    def _on_receive_state(self, state):
        self._state = state

    def reset_pose(self):
        poseCov = PoseWithCovarianceStamped()
        poseCov.pose.pose.orientation.w = 1
        self._reset_pose(poseCov)
