#!/usr/bin/env python

import math
import rospy
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from controls.msg import MoveWithSpeeds

class ToDesiredState:

    NODE_NAME = 'desired_state_movement'

    NO_STATE_MESSAGE = 'State has not yet been published, ignoring move commands'

    CONTROLS_TOPIC = '/robosub/controls/move'
    DESIRED_STATE_TOPIC = 'robosub/motion_planning/desired_state_global'
    ROBOT_STATE_TOPIC = 'robosub/state'

    DISTANCE_CUTOFF = 2
    MAX_SPEED = 0.7
    ANGLE_MAX_SPEED = 0.3
    
    def __init__(self):

        self._state = Pose()
        self._desired_state = Pose()

        self._state_received = False
        self._desired_state_received = False

        self._pub = rospy.Publisher(self.CONTROLS_TOPIC, MoveWithSpeeds, queue_size=10)
        rospy.Subscriber(self.DESIRED_STATE_TOPIC, Pose, self._receive_desired_state)
        rospy.Subscriber(self.ROBOT_STATE_TOPIC, Odometry, self._receive_robot_state)

    def _receive_desired_state(self, pose):
        self._desired_state = pose
        self._desired_state_received = True

    def _receive_robot_state(self, odom):
        self._state = odom.pose.pose
        self._state_received = True

    def run(self):

        rospy.init_node(self.NODE_NAME)

        while not rospy.is_shutdown():
            if not self._state_received:
                rospy.loginfo(self.NO_STATE_MESSAGE)
                continue

            if not self._desired_state_received:
                continue

            self._publish_speeds()

    def _publish_speeds(self):
        state_diff = self._get_state_diff(self._desired_state, self._state)

        speeds = [0] * 6
        for i in range(3):
            speeds[i] = self._get_speed_for_distance(state_diff[i])

        for i in range(3, 6):
            speeds[i] = self._get_speed_for_angle(state_diff[i])
        
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

    def _get_state_diff(self, pose1, pose2):
        """Get the difference between two poses, returning an array
        length 6 containing the difference in [x, y, z, roll, pitch, yaw]
        """
        x = pose1.position.x - pose2.position.x
        y = pose1.position.y - pose2.position.y
        z = pose1.position.z - pose2.position.z

        rpy1 = euler_from_quaternion([ pose1.orientation.x,
                                       pose1.orientation.y,
                                       pose1.orientation.z,
                                       pose1.orientation.w ])

        rpy2 = euler_from_quaternion([ pose2.orientation.x,
                                       pose2.orientation.y,
                                       pose2.orientation.z,
                                       pose2.orientation.w ])
        roll = rpy1[0] - rpy2[0]
        pitch= rpy1[1] - rpy2[1]
        yaw = rpy1[2] - rpy2[2]

        return [x, y, z, roll, pitch, yaw]



if __name__ == '__main__':
    ToDesiredState().run()
