#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
import controls.controls_utils as utils
from controls.pid.pid_manager import PIDManager
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer


class bcolors:
    BOLD = '\033[1m'
    OKGREEN = '\033[92m'
    WARN = '\033[93m'
    FAIL = '\033[91m'
    RESET = '\033[0m'


class DesiredStateHandler(Node):
    """Receives desired states and runs the corresponding controls algorithm.

    Attributes:
        listener: TransformListener used to transform input to local reference frame
        pid_manager: PIDManager object that is called for publishing data to PID loops
    """

    NODE_NAME = 'desired_state'
    DESIRED_TWIST_TOPIC = 'controls/desired_twist'
    DESIRED_POSE_TOPIC = 'controls/desired_pose'
    DESIRED_POWER_TOPIC = 'controls/desired_power'

    RUN_LOOP_RATE = 10  # for main loop

    # Max power & Max twist for the controls need values
    MAX_POWER = {'x': 1, 'y': 1, 'z': 1, 'roll': 0.5, 'pitch': 0.5, 'yaw': 0.5}
    MAX_TWIST = {'x': 1, 'y': 1, 'z': 1, 'roll': 0.5, 'pitch': 0.5, 'yaw': 0.5}

    # All variables are dictionaries that map axis to value (local reference frame)
    pose = None  # Desired pose (position)
    twist = None  # Desired twist (velocity)
    power = None  # Desired power (control effort)
    event_id = 0

    def __init__(self):
        super().__init__(self.NODE_NAME)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pid_manager = PIDManager(self)

        self.create_subscription(Pose, self.DESIRED_POSE_TOPIC, self._on_pose_received, 10)
        self.create_subscription(Twist, self.DESIRED_TWIST_TOPIC, self._on_twist_received, 10)
        self.create_subscription(Twist, self.DESIRED_POWER_TOPIC, self._on_power_received, 10)

        self.timer = self.create_timer(1/self.RUN_LOOP_RATE, self.run)

    def _on_pose_received(self, pose):
        """Handler for receiving desired pose. Transforms global desired pose to local reference
        frame for PID loops.

        Args:
            pose: ROS Pose message corresponding to desired pose in global reference frame
        """
        transform = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
        self.pose = utils.parse_pose(utils.transform_pose(pose, transform))

    def _on_twist_received(self, twist):
        """Handler for receiving desired twists. Received desired twists are assumed to be defined
        in the local reference frame.

        Args:
            power: ROS Twist message corresponding to desired twists in local reference frame
        """
        self.twist = utils.parse_twist(twist)

    def _on_power_received(self, power):
        """Handler for receiving desired powers. A desired power in a given axis represents
        the control effort that the robot should exert in a certain axis (ranges from [-1, 1]).
        Desired power is local by definition.

        Args:
            power: ROS Twist message corresponding to desired powers in local reference frame
        """
        self.power = utils.parse_twist(power)

    def _power_state_safety(self, power):
        # Compares power with controller limits
        return_status = True
        for axis in utils.get_axes():
            if abs(power[axis]) > self.MAX_POWER[axis]:
                self.get_logger().error(
                    "===> Desired power exceeds maximum %s value! Halting robot <===", axis)
                self.pid_manager.soft_estop()
                return_status = False
        return return_status

    def _twist_state_safety(self, twist):
        # Compares twist with controller limits
        return_status = True
        for axis in utils.get_axes():
            if abs(twist[axis]) > self.MAX_TWIST[axis]:
                self.get_logger().error(
                    "===> Desired twist exceeds maximum %s value! Halting robot <===", axis)
                self.pid_manager.soft_estop()
                return_status = False
        return return_status

    def _reset_data(self):
        """Resets all desired state data"""
        self.pose = None
        self.twist = None
        self.power = None

    def _validate_status(self):
        """Validates the desired state data that was received. Status is False (invalid) if multiple
        desired states are received at the same time (i.e. position and velocity control requested) 
        or if no desired state is received. If the previous status was invalid and the new status 
        is valid, increments event_id and logs a success message.

        Returns:
            True if status is valid, false otherwise
        """
        if (self.pose and self.twist) or (self.pose and self.power) or (self.twist and self.power):
            # More than one seen in one update cycle, so warn and mark as invalid
            self.pid_manager.soft_estop()
            self.get_logger().error(
                "===> Controls received conflicting desired states! Halting robot. <===")
            return False
        elif not self.pose and not self.twist and not self.power:
            if not self.pid_manager.halted:
                self.get_logger().warn(bcolors.WARN +
                                       ("===> Controls received no desired state! Halting robot. "
                                        "(Event %d) <===" % self.event_id) + bcolors.RESET)
            self.pid_manager.soft_estop()
            return False
        elif self.pid_manager.halted:
            self.get_logger().info(bcolors.OKGREEN +
                                   ("===> Controls now receiving desired state (End event %d) <===" %
                                    (self.event_id)) + bcolors.RESET)
            self.event_id += 1
        return True

    def run(self):
        if self._validate_status():
            if self.pose:
                self.pid_manager.position_control(self.pose)
            elif self.twist and self._twist_state_safety(self.twist):
                self.pid_manager.velocity_control(self.twist)
            elif self.power and self.twist_state_safety(self.power):
                self.pid_manager.power_control(self.power)
        self._reset_data()


def main(args=None):
    try:
        rclpy.init(args=args)
        desired_state = DesiredStateHandler()
        rclpy.spin(desired_state)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        raise
    finally:
        desired_state.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
