#!/usr/bin/env python3

from std_msgs.msg import Float64, Bool
from std_srvs.srv import SetBool, Empty
from geometry_msgs.msg import Pose, Twist
import controls.controls_utils as utils


class PIDManager:
    """Manages all publishing to PID loops. Pub dictionaries contain mappings from direction strings
    to corresponding publisher objects.
    """
    pub_pos = {}
    pos_enable_srv = {}
    pos_reset_srv = {}
    pub_vel = {}
    vel_enable_srv = {}
    vel_reset_srv = {}
    pub_effort = {}
    pub_power = {}
    halted = True

    def __init__(self, parent_node):
        for d in utils.get_axes():
            self.pub_pos[d] = parent_node.create_publisher(Float64, self._get_pos_topic(d), 3)
            self.pub_vel[d] = parent_node.create_publisher(Float64, self._get_vel_topic(d), 3)
            self.pub_effort[d] = parent_node.create_publisher(Float64, utils.get_effort_topic(d), 3)
            self.pub_power[d] = parent_node.create_publisher(Float64, utils.get_power_topic(d), 3)
            self.pos_enable_srv[d] = parent_node.create_client(SetBool, self._get_pos_enable_srv(d))
            self.vel_enable_srv[d] = parent_node.create_client(SetBool, self._get_vel_enable_srv(d))
            self.pos_reset_srv[d] = parent_node.create_client(Empty, self._get_pos_reset_srv(d))
            self.vel_reset_srv[d] = parent_node.create_client(Empty, self._get_vel_reset_srv(d))

    def soft_estop(self):
        """Disables all PID loops and publishes 0 to control efforts. Completely disables PID 
        control algorithm without explicitly disabling thrusters.
        """
        self._disable_loops()
        utils.publish_data_dictionary(self.pub_pos, utils.parse_pose(Pose()))
        utils.publish_data_dictionary(self.pub_vel, utils.parse_twist(Twist()))
        utils.publish_data_constant(self.pub_effort, 0.0)
        self.halted = True

    def position_control(self, pose):
        """Enables position control, which is a nested PID system. Input should be transformed to 
        the local reference frame. Position PID loops generate set-points for the velocity loops, 
        which produce control efforts.

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
        req = SetBool.Request()
        self._call_srvs(self.pos_enable_srv, False)
        utils.publish_data_dictionary(self.pub_vel, twist)

    def power_control(self, powers):
        """Enables power control, publishing control efforts directly for certain axes.
        Stabilizes on all axes with zero power required.

        Args:
            powers: A dictionary mapping direction to desired local power ranging from [-1, 1]
        """
        self._disable_loops()
        self.halted = False
        # Enable stabilization on all axes with 0 power input
        for d in powers:
            if powers[d] == 0:
                req = SetBool.Request()
                req.data = True
                self._call_srvs(self.vel_enable_srv, req, [d])
        # Publish velocity setpoints to all Velocity loops, even though some are not enabled
        utils.publish_data_dictionary(self.pub_vel, powers)
        utils.publish_data_dictionary(self.pub_power, powers)

    def _reset_loops(self):
        """ Resets pid loops, setting errors to 0 """
        req = Empty.Request()
        self._call_srvs(self.pos_reset_srv, req)
        self._call_srvs(self.vel_reset_srv, req)

    def _disable_loops(self):
        req = SetBool.Request()
        req.data = False
        self._call_srvs(self.pos_enable_srv, req)
        self._call_srvs(self.vel_enable_srv, req)

    def _enable_loops(self):
        self._reset_loops()  # Reset errors time we enable?
        req = SetBool.Request()
        req.data = True
        self._call_srvs(self.pos_enable_srv, req)
        self._call_srvs(self.vel_enable_srv, req)
        self.halted = False

    def _call_srvs(self, srvs, req, axes=utils.get_axes()):
        """ Helper method for calling services for each axis """
        futures = {}
        for axis in axes:
            futures[axis] = srvs[axis].call_async(req)
        return futures

    def _wait_for_futures(self, futures):
        """ Spin until all futures are complete. Currently not using this. """
        for axis in futures:
            rclpy.spin_until_future_complete(self, futures[axis])

    def _get_pos_topic(self, axis):
        return 'controls/setpoint/' + axis + '_pos'

    def _get_vel_topic(self, axis):
        return 'controls/setpoint/' + axis + '_vel'

    def _get_pos_enable_srv(self, axis):
        return 'controls/enable/' + axis + '_pos'

    def _get_vel_enable_srv(self, axis):
        return 'controls/enable/' + axis + '_vel'

    def _get_pos_reset_srv(self, axis):
        return 'controls/reset/' + axis + '_pos'

    def _get_vel_reset_srv(self, axis):
        return 'controls/reset/' + axis + '_vel'
