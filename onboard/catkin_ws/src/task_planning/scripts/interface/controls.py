import rospy

from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist
from custom_msgs.msg import ControlTypes
from custom_msgs.srv import SetControlTypes


class ControlsInterface:
    STATE_TOPIC = 'state'

    CONTROL_TYPES_SERVICE = 'controls/set_control_types'
    RESET_PID_LOOPS_SERVICE = 'controls/reset_pid_loops'
    DESIRED_POSITION_TOPIC = 'controls/desired_position'
    DESIRED_VELOCITY_TOPIC = 'controls/desired_velocity'

    def __init__(self, listener):
        self._listener = listener

        rospy.wait_for_service(self.CONTROL_TYPES_SERVICE)
        self._set_control_types = rospy.ServiceProxy(self.CONTROL_TYPES_SERVICE, SetControlTypes)
        # Note: if this variable gets out of sync with the actual control types,
        # bad things may happen
        self._all_axes_control_type = None

        rospy.wait_for_service(self.RESET_PID_LOOPS_SERVICE)
        self._reset_pid_loops = rospy.ServiceProxy(self.RESET_PID_LOOPS_SERVICE, Trigger)

        rospy.Subscriber(self.STATE_TOPIC, Odometry, self._on_receive_state)

        self._desired_position_pub = rospy.Publisher(self.DESIRED_POSITION_TOPIC, Pose)
        self._desired_velocity_pub = rospy.Publisher(self.DESIRED_VELOCITY_TOPIC, Twist)
        self._state = None

    def _set_all_axes_control_type(self, type):
        if self._all_axes_control_type == type:
            return
        # TODO what if this doesn't return success?
        self._set_control_types(ControlTypes(
            x=type,
            y=type,
            z=type,
            roll=type,
            pitch=type,
            yaw=type
        ))
        self._all_axes_control_type = type
        self.start_new_move()

    # Resets the PID loops. Should be called for every "new" movement
    def start_new_move(self):
        self._reset_pid_loops()

    # In global coordinates
    def publish_desired_position(self, pose):
        self._set_all_axes_control_type(ControlTypes.DESIRED_POSE)
        self._desired_position_pub.publish(pose)

    # In local coordinates
    def publish_desired_velocity(self, twist):
        self._set_all_axes_control_type(ControlTypes.DESIRED_TWIST)
        self._desired_velocity_pub.publish(twist)

    def _on_receive_state(self, state):
        self._state = state

    def get_state(self):
        return self._state
