#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64, Float32MultiArray
from nav_msgs.msg import Odometry
from custom_msgs.msg import ThrusterSpeeds
from geometry_msgs.msg import TwistStamped


class SimulationRepublisher():
    SIM_TWIST_TOPIC = 'sim/dvl'  # TwistStamped message
    SIM_IMU_TOPIC = 'sim/imu'  # Imu message
    SIM_POSE_TOPIC = 'sim/pose'  # PoseStamped message
    SENSOR_FUSION_ODOM_TOPIC = 'sensors/dvl/odom'
    SENSOR_FUSION_IMU_TOPIC = 'sensors/imu/imu'

    SIM_MOVE_TOPIC = '/sim/move'
    ROBOT_MOVE_TOPIC = '/offboard/thruster_speeds'

    def __init__(self):
        rospy.init_node('simulation_republisher')

        self.dvl_listener = rospy.Subscriber(self.SIM_TWIST_TOPIC, TwistStamped, self._on_receive_twist)
        self.imu_listener = rospy.Subscriber(self.SIM_IMU_TOPIC, Imu, self._on_receive_imu)

        self.odom_pub = rospy.Publisher(self.SENSOR_FUSION_ODOM_TOPIC, Odometry, queue_size=3)
        self.imu_pub = rospy.Publisher(self.SENSOR_FUSION_IMU_TOPIC, Imu, queue_size=3)

        self.move_pub = rospy.Publisher(self.SIM_MOVE_TOPIC, Float32MultiArray, queue_size=3)
        self.robot_listener = rospy.Subscriber(self.ROBOT_MOVE_TOPIC, ThrusterSpeeds, self._on_receive_thruster_speeds)

    def _on_receive_twist(self, twist_stamped_msg):
        """Receive TwistStamped message from simulation and assign to odom"""
        odom = Odometry()
        odom.twist.twist = twist_stamped_msg.twist
        odom.header = twist_stamped_msg.header
        odom.header.frame_id = 'odom'
        odom.child_frame_id = "dvl_link"
        odom.header.stamp.secs = rospy.get_rostime().secs
        odom.header.stamp.nsecs = rospy.get_rostime().nsecs
        self.odom_pub.publish(odom)

    def _on_receive_imu(self, imu_msg):
        """Receive imu message from simulation and publish quat for sensor fusion"""
        imu_msg.header.frame_id = "imu_link"
        imu_msg.header.stamp.secs = rospy.get_rostime().secs
        imu_msg.header.stamp.nsecs = rospy.get_rostime().nsecs
        self.imu_pub.publish(imu_msg)

    def _on_receive_thruster_speeds(self, thruster_speeds_msg):
        f32_t_allocs = Float32MultiArray()
        thruster_speeds = np.array(thruster_speeds_msg.speeds)
        f32_t_allocs.data = np.clip(thruster_speeds/127, -1, 1)
        self.move_pub.publish(f32_t_allocs)

SimulationRepublisher()
rospy.spin()
