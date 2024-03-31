import rospy

from std_srvs.srv import Trigger
from geometry_msgs.msg import Pose, Twist
from custom_msgs.msg import ControlTypes, ThrusterAllocs
from custom_msgs.srv import SetControlTypes
from utils.other_utils import singleton
import resource_retriever as rr
import os
import yaml


@singleton
class Controls:
    """
    A singleton class for the controls interface

    Attributes:
        _instance: The singleton instance of this class. Is a static attribute.
        _set_control_types: The service proxy for setting control types
        _all_axes_control_type: The control type for all axes
        _reset_pid_loop: The service proxy for resetting the PID loops
        _desired_position_pub: The publisher for the desired position
        _desired_velocity_pub: The publisher for the desired velocity
        _desired_power_pub: The publisher for the desired power
        _read_config: The config file
        num_thrusters: The number of thrusters
        thruster_dict: The thruster dictionary
        _thruster_pub: The publisher for thruster allocations
    """

    # ROS service name for setting control types
    CONTROL_TYPES_SERVICE = 'controls/set_control_types'
    RESET_PID_LOOPS_SERVICE = 'controls/reset_pid_loops'
    DESIRED_POSITION_TOPIC = 'controls/desired_position'
    DESIRED_VELOCITY_TOPIC = 'controls/desired_velocity'
    DESIRED_POWER_TOPIC = 'controls/desired_power'
    THRUSTER_ALLOCS_TOPIC = 'controls/thruster_allocs'

    def __init__(self, bypass: bool = False):

        if not bypass:
            rospy.wait_for_service(self.CONTROL_TYPES_SERVICE)
        self._set_control_types = rospy.ServiceProxy(self.CONTROL_TYPES_SERVICE, SetControlTypes)
        # Note: if this variable gets out of sync with the actual control types,
        # bad things may happen
        self._all_axes_control_type = None

        if not bypass:
            rospy.wait_for_service(self.RESET_PID_LOOPS_SERVICE)
        self._reset_pid_loops = rospy.ServiceProxy(self.RESET_PID_LOOPS_SERVICE, Trigger)

        self._desired_position_pub = rospy.Publisher(self.DESIRED_POSITION_TOPIC, Pose, queue_size=1)
        self._desired_velocity_pub = rospy.Publisher(self.DESIRED_VELOCITY_TOPIC, Twist, queue_size=1)
        self._desired_power_pub = rospy.Publisher(self.DESIRED_POWER_TOPIC, Twist, queue_size=1)

        self._read_config = None

        self.num_thrusters = None
        self.thruster_dict = None

        self.get_thruster_dict()
        self._thruster_pub = rospy.Publisher(self.THRUSTER_ALLOCS_TOPIC, ThrusterAllocs, queue_size=1)
        self.bypass = bypass

    def get_thruster_dict(self) -> None:
        """
        Get thruster dictionary

        Returns:
            The thruster dictionary
        """
        CONFIG_FILE_PATH = 'package://controls/config/%s.yaml'
        filename = rr.get_filename(CONFIG_FILE_PATH % os.getenv("ROBOT_NAME", "oogway"), use_protocol=False)
        with open(filename) as f:
            full_thruster_dict = yaml.safe_load(f)

        thruster_dict = {}
        for index, t_dict in enumerate(full_thruster_dict['thrusters']):
            thruster_name = t_dict['name']
            thruster_dict[thruster_name] = index

        self.num_thrusters = len(full_thruster_dict['thrusters'])
        self.thruster_dict = thruster_dict
        return thruster_dict

    def _set_all_axes_control_type(self, type: ControlTypes) -> None:
        """
        Set the control type for all axes

        Args:
            type: The control type to set
        """
        if self._all_axes_control_type == type:
            return
        # TODO what if this doesn't return success?
        if not self.bypass:
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
    def start_new_move(self) -> None:
        """
        Start a new movement
        """
        if not self.bypass:
            self._reset_pid_loops()

    # In global coordinates
    def publish_desired_position(self, pose: Pose) -> None:
        """
        Publish the desired position

        Args:
            pose: The desired position
        """
        self._set_all_axes_control_type(ControlTypes.DESIRED_POSITION)
        self._desired_position_pub.publish(pose)

    # In local coordinates
    def publish_desired_velocity(self, twist: Twist) -> None:
        """
        Publish the desired velocity

        Args:
            twist: The desired velocity
        """
        self._set_all_axes_control_type(ControlTypes.DESIRED_TWIST)
        self._desired_velocity_pub.publish(twist)

    def publish_desired_power(self, power: Twist) -> None:
        """
        Publish the desired power

        Args:
            power: The desired power
        """
        self._set_all_axes_control_type(ControlTypes.DESIRED_POWER)
        self._desired_power_pub.publish(power)

    def publish_thruster_allocs(self, **kwargs) -> None:
        """
        Publish the thruster allocations

        Args:
            kwargs: The thruster allocations

        Raises:
            ValueError: If the thruster name is not in thruster_dict
            ValueError: If the thruster alloc is not between -1 and 1 inclusive
        """
        thruster_allocs = [0] * self.num_thrusters

        for kwarg_name, kwarg_value in kwargs.items():

            if kwarg_name not in self.thruster_dict:
                raise ValueError(f"Thruster name not in thruster_dict {kwarg_name}")

            if kwarg_value > 1 or kwarg_value < -1:
                raise ValueError(f"Recieved {kwarg_value} for thruster {kwarg_name}. Thruster alloc must be between " +
                                 "-1 and 1 inclusive.")

            thruster_allocs[self.thruster_dict[kwarg_name]] = kwarg_value

        thruster_allocs_msg = ThrusterAllocs()
        thruster_allocs_msg.header.stamp = rospy.Time.now()
        thruster_allocs_msg.allocs = thruster_allocs

        self._thruster_pub.publish(thruster_allocs_msg)