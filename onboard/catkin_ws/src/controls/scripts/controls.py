#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool


class Controls:
    THRUSTER_ALLOCS_HZ = 20

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
        rospy.Subscriber('state', Odometry, self.recieve_state)

        # Advertise input services
        self.enable_controls_service = rospy.Service('enable_controls', SetBool, self.enable_controls_callback)
        