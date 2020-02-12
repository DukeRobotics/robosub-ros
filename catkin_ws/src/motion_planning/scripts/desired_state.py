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
from std_msgs.msg import Float64

class ToDesiredState:

    NODE_NAME = 'desired_state_movement'

    NO_STATE_MESSAGE = 'State has not yet been published, ignoring move commands'
    NO_TRANSFORM_MESSAGE = 'Could not get transform from base_link to odom, trying again'

    CONTROLS_TOPIC = 'controls/move'
    DESIRED_STATE_TOPIC = 'motion_planning/desired_state_global'

    PUB_RATE = 15
    RESET_DESIRED_STATE_TIME = 0.1  # seconds

    #DISTANCE_CUTOFF = .75
    #DISTANCE_MAX_SPEED = 0.3
    #ANGLE_CUTOFF = 0.785
    #ANGLE_MAX_SPEED = 0.2
    
    def __init__(self):

        self._desired_state = None
        self._reset_desired_state_duration = rospy.Duration(self.RESET_DESIRED_STATE_TIME)

        #self._pub = rospy.Publisher(self.CONTROLS_TOPIC, MoveWithSpeeds, queue_size=10)
        rospy.Subscriber(self.DESIRED_STATE_TOPIC, PoseStamped, self._receive_desired_state)

        # self._pid_response = [None for _ in range(6)]

        self._init_pid_publishers()
        # self._init_pid_subscribers()

    def _receive_desired_state(self, pose):
        self._desired_state = pose
        self._desired_state_time_received = rospy.Time.now()

    def _init_pid_publishers(self):
        self._pub_x = rospy.Publisher('zglobal_x/setpoint', Float64)
        self._pub_y = rospy.Publisher('global_y/setpoint', Float64)
        self._pub_z = rospy.Publisher('global_z/setpoint', Float64)

        self._pub_roll = rospy.Publisher('global_roll/setpoint', Float64)
        self._pub_pitch = rospy.Publisher('global_pitch/setpoint', Float64)
        self._pub_yaw = rospy.Publisher('global_yaw/setpoint', Float64)

    # def _init_pid_subscribers(self):
    #     rospy.Subscriber('global_x/control_effort', Float64, self._on_receive_gx)
    #     rospy.Subscriber('global_y/control_effort', Float64, self._on_receive_gy)
    #     rospy.Subscriber('global_z/control_effort', Float64, self._on_receive_gz)
    #
    #     rospy.Subscriber('global_roll/control_effort', Float64, self._on_receive_groll)
    #     rospy.Subscriber('global_pitch/control_effort', Float64, self._on_receive_gpitch)
    #     rospy.Subscriber('global_yaw/control_effort', Float64, self._on_receive_gyaw)

    # def _on_receive_gx(self, gx):
    #     self._pid_response[0] = gx
    #
    # def _on_receive_gy(self, gy):
    #     self._pid_response[1] = gy
    #
    # def _on_receive_gz(self, gz):
    #     self._pid_response[2] = gz
    #
    # def _on_receive_groll(self, groll):
    #     self._pid_response[3] = groll
    #
    # def _on_receive_gpitch(self, gpitch):
    #     self._pid_response[4] = gpitch
    #
    # def _on_receive_gyaw(self, gyaw):
    #     self._pid_response[5] = gyaw

    def run(self):

        rospy.init_node(self.NODE_NAME)
        self._desired_state_time_received = rospy.Time.now()

        self._rate = rospy.Rate(self.PUB_RATE)

        self._tfBuffer = tf2_ros.Buffer()
        self._tfListener = tf2_ros.TransformListener(self._tfBuffer);

        while not rospy.is_shutdown():

            if self._desired_state is None:
                continue

            self._to_robot_transform = self._get_robot_transform()

            #if rospy.Time.now() - self._desired_state_time_received > self._reset_desired_state_duration:
            #    self._desired_state = tf2_geometry_msgs.do_transform_pose(PoseStamped(),
            #                                                              self._to_robot_transform)

            local_desired_pose = tf2_geometry_msgs.do_transform_pose(self._desired_state,
                                                                     self._to_robot_transform)
            pprint('LOCAL')
            pprint(local_desired_pose)
            # self._reset_received_values()
            self._publish_pid_setpoints(local_desired_pose)
            # self._wait_for_pid_response()
            # self._publish_pid_response()
            
            self._rate.sleep()

    def _get_robot_transform(self):
        """Try and get the transform from odom to base_link
        """
        while True:
            try:
                return self._tfBuffer.lookup_transform('base_link', 'odom', rospy.Time(0), rospy.Duration(0.5))
            except:
                rospy.logerr(self.NO_TRANSFORM_MESSAGE)

    def _reset_received_values(self):
        self._pid_response = [None for _ in range(6)]

    def _publish_pid_setpoints(self, local_pose):
        self._pub_x.publish(local_pose.pose.position.x)
        self._pub_y.publish(local_pose.pose.position.y)
        self._pub_z.publish(local_pose.pose.position.z)

        rpy = euler_from_quaternion([local_pose.pose.orientation.x,
                                     local_pose.pose.orientation.y,
                                     local_pose.pose.orientation.z,
                                     local_pose.pose.orientation.w])

        self._pub_roll.publish(rpy[0])
        self._pub_pitch.publish(rpy[1])
        self._pub_yaw.publish(rpy[2])

    # def _wait_for_pid_response(self):
    #     rate = rospy.Rate(10)
    #     while not rospy.is_shutdown():
    #         if None not in self._pid_response:
    #             break
    #         rate.sleep()
    #
    # def _publish_pid_response(self):
    #     self._pub.pub.publish(self._pid_response)

    # def _publish_speeds(self):
    #     local_desired_pose = tf2_geometry_msgs.do_transform_pose(self._desired_state,
    #                                                              self._to_robot_transform)
    #
    #     speeds = [0] * 6
    #
    #     speeds[0] = self._get_speed_for_distance(local_desired_pose.pose.position.x)
    #     speeds[1] = self._get_speed_for_distance(local_desired_pose.pose.position.y)
    #     speeds[2] = self._get_speed_for_distance(local_desired_pose.pose.position.z)
    #
    #     rpy = euler_from_quaternion([ local_desired_pose.pose.orientation.x,
    #                                   local_desired_pose.pose.orientation.y,
    #                                   local_desired_pose.pose.orientation.z,
    #                                   local_desired_pose.pose.orientation.w ])
    #
    #     speeds[3] = self._get_speed_for_angle(rpy[0])
    #     speeds[4] = self._get_speed_for_angle(rpy[1])
    #     speeds[5] = self._get_speed_for_angle(rpy[2])
    #
    #     move_msg = MoveWithSpeeds(speeds)
    #     self._pub.publish(move_msg)
    #
    # def _get_speed_for_distance(self, distance):
    #     """Decide the speed, between -1 and 1 based on the distance
    #     """
    #     sign = (distance > 0) - (distance < 0)
    #     if abs(distance) > self.DISTANCE_CUTOFF:
    #         return self.DISTANCE_MAX_SPEED * sign
    #     else:
    #         return (distance / self.DISTANCE_CUTOFF) * self.DISTANCE_MAX_SPEED
    #
    # def _get_speed_for_angle(self, angle):
    #     sign = (angle > 0) - (angle < 0)
    #     if abs(angle) > self.ANGLE_CUTOFF:
    #         return self.ANGLE_MAX_SPEED * sign
    #     else:
    #         return (angle / (self.ANGLE_CUTOFF)) * self.ANGLE_MAX_SPEED


if __name__ == '__main__':
    ToDesiredState().run()
