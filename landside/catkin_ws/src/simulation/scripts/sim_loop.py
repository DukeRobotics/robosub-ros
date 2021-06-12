#!/usr/bin/env python3

import rospy
from cthulhu_model import Cthulhu
from custom_msgs.msg import ThrusterSpeeds
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from sim_handle import SimHandle
from std_msgs.msg import Float64


class SimLoop:

    ODOM_TOPIC = 'sensors/dvl/odom'
    IMU_TOPIC = 'sensors/imu/imu'
    ROBOT_MOVE_TOPIC = 'offboard/thruster_speeds'
    DEPTH_TOPIC = 'sensors/depth'

    def __init__(self):
        rospy.init_node("simulation")

        self.sim_handle = SimHandle()
        self.robot_model = Cthulhu(self.sim_handle.get_mass())
        self.sub = rospy.Subscriber(self.ROBOT_MOVE_TOPIC, ThrusterSpeeds, self.on_move_received)

        self.odom_pub = rospy.Publisher(self.ODOM_TOPIC, Odometry, queue_size=3)
        self.imu_pub = rospy.Publisher(self.IMU_TOPIC, Imu, queue_size=3)
        self.depth_pub = rospy.Publisher(self.DEPTH_TOPIC, Float64, queue_size=3)

        self.on_move_received(ThrusterSpeeds())

    def on_move_received(self, msg):
        self.tforces = self.robot_model.get_thruster_forces(msg.speeds)

    def publish_imu(self, pose, twist):
        msg = Imu()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'imu_link'
        msg.orientation = pose.orientation
        msg.angular_velocity = twist.angular
        self.imu_pub.publish(msg)

    def publish_odom(self, pose, twist):
        msg = Odometry()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'dvl_link'
        msg.pose.pose = pose
        msg.twist.twist = twist
        self.odom_pub.publish(msg)

    def publish_depth(self, pose, twist):
        msg = Float64()
        # Float64 doesn't have a header for timestamps or fields for
        # parent/child frame labels, so they aren't included.
        msg.data = -pose.position.z  # Assume position is in meters
        self.depth_pub.publish(msg)

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            pose = self.sim_handle.get_pose()
            twist = self.sim_handle.get_twist()

            self.publish_imu(pose, twist)
            self.publish_odom(pose, twist)
            self.publish_depth(pose, twist)
            self.sim_handle.set_thruster_force(self.tforces)

            rate.sleep()


if __name__ == '__main__':
    SimLoop().run()
