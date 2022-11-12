#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Pose, Twist
import controls_utils as utils


class PIDManager:
    """Manages all publishing to PID loops. Pub dictionaries contain mappings from direction strings
    to corresponding rospy publisher objects.
    """
    pub_pos = {}
    pub_pos_enable = {}
    pub_vel = {}
    pub_vel_enable = {}
    pub_control_effort = {}
    pub_power = {}

    def __init__(self):
        for d in utils.get_axes():
            self.pub_pos[d] = rospy.Publisher(self._get_pos_pid_topic(d), Float64, queue_size=3)
            self.pub_pos_enable[d] = rospy.Publisher(self._get_pos_pid_enable_topic(d), Bool, queue_size=3)
            self.pub_vel[d] = rospy.Publisher(self._get_vel_pid_topic(d), Float64, queue_size=3)
            self.pub_vel_enable[d] = rospy.Publisher(self._get_vel_pid_enable_topic(d), Bool, queue_size=3)
            self.pub_control_effort[d] = rospy.Publisher(utils.get_controls_move_topic(d), Float64, queue_size=3)
            self.pub_power[d] = rospy.Publisher(utils.get_power_topic(d), Float64, queue_size=3)

    def soft_estop(self):
        """Disables all PID loops and publishes 0 to control efforts. Completely disables PID control
        algorithm without explicitly disabling thrusters.
        """
        self._disable_loops()
        utils.publish_data_dictionary(self.pub_pos, utils.parse_pose(Pose()))
        utils.publish_data_dictionary(self.pub_vel, utils.parse_twist(Twist()))
        utils.publish_data_constant(self.pub_control_effort, 0)

    def position_control(self, pose):
        """Enables position control, which is a nested PID system. Input should be transformed to the
        local reference frame. Position PID loops generate set-points for the velocity loops, which
        produce control efforts.

        Args:
            pose: A dictionary mapping direction to desired local pose
        """
        self._enable_loops()
        utils.publish_data_dictionary(self.pub_pos, pose)

    def velocity_control(self, twist):
        """Enables velocity control, bypassing position loops. Input should be transformed to the
        local reference frame. Velocity PID loops produce control efforts based on desired twist.

        Args:
            twist: A dictionary mapping direction to desired local velocity
        """
        self._enable_loops()
        # Disable position loop
        utils.publish_data_constant(self.pub_pos_enable, False)
        utils.publish_data_dictionary(self.pub_vel, twist)

    def power_control(self, powers):
        """Enables power control, publishing control efforts directly for certain axes.
        Stabilizes on all axes with zero power required.

        Args:
            powers: A dictionary mapping direction to desired local power ranging from [-1, 1]
        """
        self._disable_loops()
        # Enable stabilization on all axes with 0 power input
        for d in powers:
            if powers[d] == 0:
                utils.publish_data_constant(self.pub_vel_enable, True, [d])
        # Publish velocity setpoints to all Velocity loops, even though some are not enabled
        utils.publish_data_dictionary(self.pub_vel, powers)
        utils.publish_data_dictionary(self.pub_power, powers)

    def _disable_loops(self):
        utils.publish_data_constant(self.pub_pos_enable, False)
        utils.publish_data_constant(self.pub_vel_enable, False)

    def _enable_loops(self):
        utils.publish_data_constant(self.pub_pos_enable, True)
        utils.publish_data_constant(self.pub_vel_enable, True)

    def _get_pos_pid_topic(self, axis):
        return 'controls/' + axis + '_pos/setpoint'

    def _get_vel_pid_topic(self, axis):
        return 'controls/' + axis + '_vel/setpoint'

    def _get_pos_pid_enable_topic(self, axis):
        return 'controls/enable/' + axis + '_pos'

    def _get_vel_pid_enable_topic(self, axis):
        return 'controls/enable/' + axis + '_vel'