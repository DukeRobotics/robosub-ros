#!/usr/bin/env python

import rospy
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import tf2_ros
import tf2_geometry_msgs
import numpy as np

from geometry_msgs.msg import PoseStamped, TwistStamped
# from nav_msgs.msg import Odometry


class DesiredStateHandler():

    DESIRED_TWIST_TOPIC = ''  # TBD - where does this info come from
    DESIRED_POSE_TOPIC = ''  # TBD - where does this info come from
    PUBLISHING_TOPIC_X = 'global_x/setpoint'
    PUBLISHING_TOPIC_Y = 'global_y/setpoint'
    PUBLISHING_TOPIC_Z = 'global_z/setpoint'
    PUBLISHING_TOPIC_ROLL = 'global_roll/setpoint'
    PUBLISHING_TOPIC_PITCH = 'global_pitch/setpoint'
    PUBLISHING_TOPIC_YAW = 'global_yaw/setpoint' 

    pose_stamped = None
    twist_stamped = None

    def __init__(self):
        self._pub_x = rospy.Publisher(self.PUBLISHING_TOPIC_X, Float64, queue_size=3)
        self._pub_y = rospy.Publisher(self.PUBLISHING_TOPIC_Y, Float64, queue_size=3)
        self._pub_z = rospy.Publisher(self.PUBLISHING_TOPIC_Z, Float64, queue_size=3)
        self._pub_roll = rospy.Publisher(self.PUBLISHING_TOPIC_ROLL, Float64, queue_size=3)
        self._pub_pitch = rospy.Publisher(self.PUBLISHING_TOPIC_PITCH, Float64, queue_size=3)
        self._pub_yaw = rospy.Publisher(self.PUBLISHING_TOPIC_YAW, Float64, queue_size=3)

        rospy.Subscriber(self.DESIRED_POSE_TOPIC, PoseStamped, self.receive_pose_stamped)
        rospy.Subscriber(self.DESIRED_TWIST_TOPIC, TwistStamped, self.receive_twist_stamped)

    def receive_pose_stamped(self, pose_stamped):
        self.pose_stamped = pose_stamped

    def receive_twist_stamped(self, twist_stamped):
        self.twist_stamped = twist_stamped

    def run(self):
        rospy.init_node('desired_state')
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            if not self.pose_stamped and not self.twist_stamped:
                # TODO: If it's been too long, stop PID and warn (timeout)
                continue
            if self.pose_stamped and self.twist_stamped:
                # TODO: Both seen in one update cycle, so warn and continue
                continue

            # Now we have either pose stamped XOR twist stamped
            if self.pose_stamped:
                x = self.pose_stamped.pose.position.x
                y = self.pose_stamped.pose.position.y
                z = self.pose_stamped.pose.position.z

                orientation = self.pose_stamped.pose.orientation
                roll, pitch, yaw = euler_from_quaternion([orientation.x,
                                                          orientation.y,
                                                          orientation.z,
                                                          orientation.w])

                if self.pose_stamped.header.frame_id == 'local':
                    # TODO: implement transformation to global
                    pass

                self._pub_x.publish(x)
                self._pub_y.publish(y)
                self._pub_z.publish(z)
                self._pub_roll.publish(roll)
                self._pub_pitch.publish(pitch)
                self._pub_yaw.publish(yaw)

                self.pose_stamped = None
            elif self.twist_stamped:
                # TODO: implement this stuff lol
                self.twist_stamped = None

            rate.sleep()


def main():
    DesiredStateHandler().run()

if __name__ == '__main__':
    main()
