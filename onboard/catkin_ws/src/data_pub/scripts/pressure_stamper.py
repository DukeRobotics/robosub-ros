#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped


class PressureStamper:

    NODE_NAME = "depth_pub"
    PRESSURE_SUB_TOPIC = "offboard/pressure"
    DEPTH_DEST_TOPIC = "sensors/depth"

    FILTER_CONSTANT = 5

    def __init__(self):
        rospy.init_node(self.NODE_NAME)
        self._pub_depth = rospy.Publisher(self.DEPTH_DEST_TOPIC, PoseWithCovarianceStamped, queue_size=50)
        self._sub_pressure = rospy.Subscriber(self.PRESSURE_SUB_TOPIC, PoseWithCovarianceStamped, self.receive_pressure)
        rospy.spin()

    def receive_pressure(self, pressure):
        pressure.header.stamp = rospy.Time.now()
        pressure.pose.pose.position.z = -pressure.pose.pose.position.z

        if pressure.pose.pose.position.z <= self.FILTER_CONSTANT:
            self._pub_depth.publish(pressure)


if __name__ == '__main__':
    try:
        PressureStamper()
    except rospy.ROSInterruptException:
        pass
