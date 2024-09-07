import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from robot_localization.srv import SetPose
from sensor_msgs.msg import Imu
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
    DEPTH_TOPIC = '/sensors/depth'
    IMU_TOPIC = '/vectornav/IMU'
    RESET_POSE_SERVICE = '/set_pose'

    def __init__(self, bypass: bool = False, tfBuffer: Buffer = None):
        """
        Args:
            tfBuffer: The transform buffer for the robot
        """
        self.bypass = bypass
        if tfBuffer:
            self._tfBuffer = tfBuffer

        self.received_state = False
        self.received_depth = False
        self.received_imu = False

        rospy.Subscriber(self.STATE_TOPIC, Odometry, self._on_receive_state)
        self._state = None

        if not bypass:
            rospy.wait_for_service(self.RESET_POSE_SERVICE)
        self._reset_pose = rospy.ServiceProxy(self.RESET_POSE_SERVICE, SetPose)

        rospy.Subscriber(self.DEPTH_TOPIC, PoseWithCovarianceStamped, self._on_receive_depth)

        rospy.Subscriber(self.IMU_TOPIC, Imu, self._on_receive_imu)

    @property
    def state(self):
        """
        The state
        """
        return self._state

    @property
    def orig_state(self):
        """
        The first state message received
        """
        return self._orig_state

    @property
    def depth(self):
        """
        The depth from the pressure sensor
        """
        return self._depth

    @property
    def orig_depth(self):
        """
        The depth from the pressure sensor
        """
        return self._orig_depth

    @property
    def imu(self):
        """
        The IMU data
        """
        return self._imu

    @property
    def orig_imu(self):
        """
        The first IMU message received
        """
        return self._orig_imu

    @property
    def tfBuffer(self):
        """
        The transform buffer
        """
        return self._tfBuffer

    def _on_receive_state(self, state):
        self._state = state

        if not self.received_state:
            self._orig_state = state
            self.received_state = True

    def _on_receive_depth(self, depth_msg):
        self._depth = depth_msg.pose.pose.position.z

        if not self.received_depth:
            self._orig_depth = depth_msg.pose.pose.position.z
            self.received_depth = True

    def _on_receive_imu(self, imu_msg):
        self._imu = imu_msg

        if not self.received_imu:
            self._orig_imu = imu_msg
            self.received_imu = True

    def reset_pose(self):
        """
        Reset the pose
        """
        poseCov = PoseWithCovarianceStamped()
        poseCov.pose.pose.orientation.w = 1
        if not self.bypass:
            self._reset_pose(poseCov)
