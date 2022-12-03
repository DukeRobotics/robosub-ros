import rospy
import task_utils
import actionlib

from nav_msgs.msg import Odometry
from custom_msgs.msg import ControlsDesiredPoseAction, ControlsDesiredTwistAction, \
    ControlsDesiredPoseGoal, ControlsDesiredTwistGoal


class ControlsInterface:
    DESIRED_POSE_ACTION = 'controls/desired_pose'
    DESIRED_TWIST_ACTION = 'controls/desired_twist'

    def __init__(self, listener):
        self.listener = listener
        
        rospy.Subscriber(self.STATE_TOPIC, Odometry, self._on_receive_state)
        self.desired_pose_client = actionlib.SimpleActionClient(
            self.DESIRED_POSE_ACTION, ControlsDesiredPoseAction)
        self.desired_twist_client = actionlib.SimpleActionClient(
            self.DESIRED_TWIST_ACTION, ControlsDesiredTwistAction)
        self.state = None

        self.desired_pose_client.wait_for_server()
        self.desired_twist_client.wait_for_server()

    def move_to_pose_global(self, pose):
        self.desired_pose_client.send_goal(ControlsDesiredPoseGoal(pose=pose))

    def move_to_pose_local(self, pose):
        self.move_to_pose_global(task_utils.transform_pose(self.listener, 'base_link', 'odom', pose))

    def move_with_velocity(self, twist):
        self.desired_twist_client.send_goal(ControlsDesiredTwistGoal(twist=twist))

    def _on_receive_state(self, state):
        self.state = state

    def get_state(self):
        return self.state
