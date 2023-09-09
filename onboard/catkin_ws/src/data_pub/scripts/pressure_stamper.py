#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float64


class PressureStamper:

    NODE_NAME = "depth_pub"
    DEPTH_DEST_TOPIC = "sensors/depth"
    PRESSURE_SUB_TOPIC = "offboard/pressure"

    FILTER_CONSTANT = 6

    pose = PoseWithCovarianceStamped()

    def __init__(self):
        rospy.init_node(self.NODE_NAME)
        self._pub_depth = rospy.Publisher(self.DEPTH_DEST_TOPIC, PoseWithCovarianceStamped, queue_size=50)
        self._sub_pressure = rospy.Subscriber(self.PRESSURE_SUB_TOPIC, Float64, self.receive_pressure)
        rospy.spin()

    def receive_pressure(self, data):

        self.pose.header.frame_id = "odom"

        self.pose.pose.pose.position.x = 0.0
        self.pose.pose.pose.position.y = 0.0
        self.pose.pose.pose.position.z = -1 * data.data

        self.pose.pose.pose.orientation.x = 0.0
        self.pose.pose.pose.orientation.y = 0.0
        self.pose.pose.pose.orientation.z = 0.0
        self.pose.pose.pose.orientation.w = 1.0

        self.pose.pose.covariance[14] = 0.01

        self.pose.header.stamp = rospy.Time.now()

        if self.pose.pose.pose.position.z <= self.FILTER_CONSTANT:
            self._pub_depth.publish(self.pose)


if __name__ == '__main__':
    try:
        PressureStamper()
    except rospy.ROSInterruptException:
        pass
