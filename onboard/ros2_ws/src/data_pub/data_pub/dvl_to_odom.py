#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
import numpy as np
from custom_msgs.msg import DVLRaw
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler


class DVLOdomPublisher(Node):

    NODE_NAME = 'dvl_odom_pub'
    DVL_RAW_TOPIC = 'sensors/dvl/raw'
    DVL_ODOM_TOPIC = 'sensors/dvl/odom'

    DVL_BAD_STATUS_MSG = 'V'

    def __init__(self):
        super().__init__(self.NODE_NAME)
        self.odom_pub = self.create_publisher(Odometry, self.DVL_ODOM_TOPIC, 50)
        self.create_subscription(DVLRaw, self.DVL_RAW_TOPIC, self._republish_odom, 50)

    def _republish_odom(self, msg):
        # check if the data is good
        # only check bs and sa status they are the only two data that we use
        # there is no status for sa
        # for status: A = good, V = bad
        if msg.bs_status == self.DVL_BAD_STATUS_MSG:
            return
        # handle message here
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'

        # Position data does not exist, is set to 0 here and should not be used
        x = 0
        y = 0
        z = 0

        # bs velocity, normalized to meters (given in mm)
        vx = -np.float64(msg.bs_longitudinal) / 1000
        vy = np.float64(msg.bs_transverse) / 1000
        vz = np.float64(msg.bs_normal) / 1000

        # quat
        roll = math.radians(np.float64(msg.sa_roll))
        pitch = math.radians(np.float64(msg.sa_pitch))
        yaw = math.radians(np.float64(msg.sa_heading))
        odom_quat = quaternion_from_euler(roll, pitch, yaw)

        # set pose
        odom.pose.pose = Pose(Point(x, y, z), Quaternion(*odom_quat))
        odom.child_frame_id = "dvl_link"
        # set twist (set angular velocity to (0, 0, 0), should not be used)
        odom.twist.twist = Twist(Vector3(vx, vy, vz), Vector3(0, 0, 0))
        self.odom_pub.publish(odom)


def main(args=None):
    try:
        rclpy.init(args=args)
        dvl_odom = DVLOdomPublisher()
        rclpy.spin(dvl_odom)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        raise
    finally:
        dvl_odom.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
