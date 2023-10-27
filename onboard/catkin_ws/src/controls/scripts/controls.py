#!/usr/bin/env python3

import rospy
import yaml

from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool
from std_msgs.msg import Bool
from custom_msgs.msg import ThrusterAllocs, PIDGain, PIDGains, ControlTypes

from pid import PID
from thruster_allocator import ThrusterAllocator
import controls_utils as utils


class Controls:
    THRUSTER_ALLOCS_RATE = rospy.Rate(20)

    def __init__(self):
        rospy.init_node('controls')

        # Get parameters from launch file
        self.debug = rospy.get_param('~debug', False)
        self.position_pid = rospy.get_param('~position_pid', False)
        self.velocity_pid = rospy.get_param('~velocity_pid', False)

        # Subscribe to input topics
        rospy.Subscriber('controls/desired_position', Pose, self.desired_pose_callback)
        rospy.Subscriber('controls/desired_velocity', Twist, self.desired_twist_callback)
        rospy.Subscriber('controls/desired_power', Twist, self.desired_power_callback)
        rospy.Subscriber('state', Odometry, self.state_callback)

        # Advertise input services
        self.enable_controls_service = rospy.Service('enable_controls', SetBool, self.enable_controls_callback)
        self.set_control_types_service = rospy.Service('set_control_types', SetBool, self.set_control_types_callback)
        self.set_pid_gains_service = rospy.Service('set_pid_gains', SetBool, self.set_pid_gains_callback)

        # Initilize publishers for output topics
        self.thruster_allocs_pub = rospy.Publisher('controls/thruster_allocs', ThrusterAllocs, queue_size=1)
        self.desired_thruster_allocs_pub = rospy.Publisher('controls/desired_thruster_allocs', ThrusterAllocs,
                                                           queue_size=1)
        self.set_power_pub = rospy.Publisher('controls/set_power', Twist, queue_size=1)
        self.pid_gains_pub = rospy.Publisher('controls/pid_gains', PIDGains, queue_size=1)
        self.control_types_pub = rospy.Publisher('controls/control_types', ControlTypes, queue_size=1)
        self.position_error_pub = rospy.Publisher('controls/position_error', Pose, queue_size=1)
        self.velocity_error_pub = rospy.Publisher('controls/velocity_error', Twist, queue_size=1)
        self.status_pub = rospy.Publisher('controls/status', Bool, queue_size=1)

        # Initialize PID controllers
        self.pid_loops = {}
        self.pid_gains = []
        self.init_pid_controllers()

        # Initialize thruster allocator
        self.thruster_allocator = ThrusterAllocator()

    def init_pid_controllers(self):
        """
        Initializes all PID controllers. The PID gains are retrieved from the config file for the current robot
        based on the ROBOT_NAME environment variable and the PID controllers are initialized with these gains.

        Returns:
            None
        """
        pid_config_filename = utils.get_config_file(utils.ConfigFileType.PID)
        with open(pid_config_filename) as f:
            pid_config_file = yaml.safe_load(f)

        for pid_loop in pid_config_file:
            self.pid_loops[pid_loop] = {}

            for axis in pid_loop:
                self.pid_loops[pid_loop][axis] = PID(axis['Kp'], axis['Ki'], axis['Kd'], axis['Ff'])

    def desired_pose_callback(self, desired_pose):
        pass

    def desired_twist_callback(self, desired_twist):
        pass

    def desired_power_callback(self, desired_power):
        pass

    def enable_controls_callback(self, req):
        pass

    def set_control_types_callback(self, req):
        pass

    def set_pid_gains_callback(self, req):
        pass

    def state_callback(self, state):
        pass

    def run(self):
        while not rospy.is_shutdown():

            pid_gains = []
            for pid_loop in utils.PID_LOOPS:
                for axis in utils.AXES:
                    pid_gain = PIDGain()
                    pid_gain.pid_loop = getattr(pid_gain, f"{pid_loop.upper()}_PID")
                    pid_gain.axis = getattr(pid_gain, f"AXIS_{axis.upper()}")
                    self.pid_loops[pid_loop][axis] = PID(axis['Kp'], axis['Ki'], axis['Kd'], axis['Ff'])

            self.THRUSTER_ALLOCS_RATE.sleep()


if __name__ == '__main__':
    controls = Controls()
    controls.run()
