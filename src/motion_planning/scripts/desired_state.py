#!/usr/bin/env python

import math
import rospy
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import tf2_ros
import tf2_geometry_msgs
import numpy as np
from pprint import pprint

from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from controls.msg import MoveWithSpeeds

class ToDesiredState:

    NODE_NAME = 'desired_state_movement'

    NO_STATE_MESSAGE = 'State has not yet been published, ignoring move commands'
    NO_TRANSFORM_MESSAGE = 'Could not get transform from base_link to odom, trying again'

    CONTROLS_TOPIC = 'controls/move'
    DESIRED_STATE_TOPIC = 'motion_planning/desired_state_global'

    PUB_RATE = 15

    DISTANCE_CUTOFF = .75
    DISTANCE_MAX_SPEED = 0.3
    ANGLE_CUTOFF = 0.785
    ANGLE_MAX_SPEED = 0.2
    
    def __init__(self):

        self._desired_state = PoseStamped()

        self._desired_state_received = False

        self._pub = rospy.Publisher(self.CONTROLS_TOPIC, MoveWithSpeeds, queue_size=10)
        rospy.Subscriber(self.DESIRED_STATE_TOPIC, PoseStamped, self._receive_desired_state)

    def _receive_desired_state(self, pose):
        self._desired_state = pose
        self._desired_state_received = True

    def run(self):

        rospy.init_node(self.NODE_NAME)

        self._rate = rospy.Rate(self.PUB_RATE)

        self._tfBuffer = tf2_ros.Buffer()
        self._tfListener = tf2_ros.TransformListener(self._tfBuffer);

        while not rospy.is_shutdown():

            if not self._desired_state_received:
                continue

            self._to_robot_transform = self._get_robot_transform()

            self._publish_speeds()
            self._rate.sleep()

    def _get_robot_transform(self):
        """Try and get the transform from odom to base_link
        """
        while True:
            try:
                return self._tfBuffer.lookup_transform('base_link', 'odom',                                                    rospy.Time(0), rospy.Duration(0.5))
            except:
                rospy.logerr(self.NO_TRANSFORM_MESSAGE)

    def _publish_speeds(self):
        local_desired_pose = tf2_geometry_msgs.do_transform_pose(self._desired_state,
                                                                 self._to_robot_transform)
        pprint(local_desired_pose)

        speeds = [0] * 6

        speeds[0] = self._get_speed_for_distance(local_desired_pose.pose.position.x)
        speeds[1] = self._get_speed_for_distance(local_desired_pose.pose.position.y)
        speeds[2] = self._get_speed_for_distance(local_desired_pose.pose.position.z)

        rpy = euler_from_quaternion([ local_desired_pose.pose.orientation.x,
                                      local_desired_pose.pose.orientation.y,
                                      local_desired_pose.pose.orientation.z,
                                      local_desired_pose.pose.orientation.w ])

        speeds[3] = self._get_speed_for_angle(rpy[0])
        speeds[4] = self._get_speed_for_angle(rpy[1])
        speeds[5] = self._get_speed_for_angle(rpy[2])
        
        move_msg = MoveWithSpeeds(speeds)
        self._pub.publish(move_msg)

    def _get_speed_for_distance(self, distance):
        """Decide the speed, between -1 and 1 based on the distance
        """
        sign = np.sign(distance)  #(distance > 0) - (distance < 0)
        if abs(distance) > self.DISTANCE_CUTOFF:
            return self.DISTANCE_MAX_SPEED * sign
        else:
            return (distance / self.DISTANCE_CUTOFF) * self.DISTANCE_MAX_SPEED

    def _get_speed_for_angle(self, angle):
        sign = np.sign(angle)  #(angle > 0) - (angle < 0)
        if abs(angle) > self.ANGLE_CUTOFF:
            return self.ANGLE_MAX_SPEED * sign
        else:
            return (angle / (self.ANGLE_CUTOFF)) * self.ANGLE_MAX_SPEED


if __name__ == '__main__':
    ToDesiredState().run()
