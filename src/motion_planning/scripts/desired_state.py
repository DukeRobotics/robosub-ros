#!/usr/bin/env python

import math
import rospy
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import tf2_ros
import tf2_geometry_msgs

from geometry_msgs.msg import Pose, TransformStamped
from nav_msgs.msg import Odometry
from controls.msg import MoveWithSpeeds

class ToDesiredState:

    NODE_NAME = 'desired_state_movement'

    NO_STATE_MESSAGE = 'State has not yet been published, ignoring move commands'

    CONTROLS_TOPIC = 'controls/move'
    DESIRED_STATE_TOPIC = 'motion_planning/desired_state_global'

    PUB_RATE = 10

    DISTANCE_CUTOFF = 2
    MAX_SPEED = 0.7
    ANGLE_MAX_SPEED = 0.3
    
    def __init__(self):

        self._desired_state = Pose()

        self._desired_state_received = False

        self._pub = rospy.Publisher(self.CONTROLS_TOPIC, MoveWithSpeeds, queue_size=10)
        rospy.Subscriber(self.DESIRED_STATE_TOPIC, Pose, self._receive_desired_state)

        self._rate = rospy.Rate(self.PUB_RATE)

    def _receive_desired_state(self, pose):
        self._desired_state = pose
        self._desired_state_received = True

    def run(self):

        rospy.init_node(self.NODE_NAME)

        self._tfBuffer = tf2_ros.Buffer()
        self._tfListener = tf2_ros.TransformListener(self._tfBuffer);
        self._tfListener.wait_for_transform('base_link', 'odom',
                                            rospy.Time().now(), rospy.Duration.from_secs(100))
        self._to_robot_transform = \
                 self._tfBuffer.lookup_transform('base_link', 'odom',                                                    rospy.Time(0), rospy.Duration(0.5))

        while not rospy.is_shutdown():

            if not self._desired_state_received:
                continue

            self._publish_speeds()
            self._rate.sleep()

    def _publish_speeds(self):
        local_desired_pose = tf2_geometry_msgs.do_transform_pose(self._desired_state,
                                                                 self._to_robot_transform)

        speeds = [0] * 6

        speeds[0] = self._get_speed_for_distance(local_desired_pose.position.x)
        speeds[1] = self._get_speed_for_distance(local_desired_pose.position.y)
        speeds[2] = self._get_speed_for_distance(local_desired_pose.position.z)

        rpy = euler_from_quaternion([ local_desired_pose.orientation.x,
                                      local_desired_pose.orientation.y,
                                      local_desired_pose.orientation.z,
                                      local_desired_pose.orientation.w ])

        speeds[3] = self._get_speed_for_angle(rpy[0])
        speeds[4] = self._get_speed_for_angle(rpy[1])
        speeds[5] = self._get_speed_for_angle(rpy[2])
        
        move_msg = MoveWithSpeeds(speeds)
        self._pub.publish(move_msg)

    def _get_speed_for_distance(self, distance):
        """Decide the speed, between -1 and 1 based on the distance
        """
        sign = (distance > 0) - (distance < 0)
        if abs(distance) > self.DISTANCE_CUTOFF:
            return self.MAX_SPEED * sign
        else:
            return (distance / self.DISTANCE_CUTOFF) * self.MAX_SPEED

    def _get_speed_for_angle(self, angle):
        sign = (angle > 0) - (angle < 0)
        if abs(angle) > self.ANGLE_MAX_SPEED:
            return self.ANGLE_MAX_SPEED * sign
        else:
            return (angle / 2 * math.pi) * self.ANGLE_MAX_SPEED


if __name__ == '__main__':
    ToDesiredState().run()
