#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool
from std_msgs.msg import Bool
from custom_msgs.msg import ThrusterAllocs, PIDGains, ControlTypes

from pid import PID
from thruster_allocator import ThrusterAllocator

class Controls:
    THRUSTER_ALLOCS_HZ = 20
    AXES = ['x', 'y', 'z', 'roll', 'pitch', 'yaw']

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

        # TODO: Initialize PID controllers
        # Get PID gains from config file
        # Initialize PID controllers in dictionary below
        self.position_pid = {}

        # TODO: Initialize thruster allocator with config file name
        self.thruster_allocator = ThrusterAllocator()

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
        pass


if __name__ == '__main__':
    controls = Controls()
    controls.run()
