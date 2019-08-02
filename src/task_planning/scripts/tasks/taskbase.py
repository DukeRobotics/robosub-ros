#!/usr/bin/env python

from enum import Enum
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import rospy
import numpy as np

class TaskBase(object):

    CONTINUE = 1
    FINISHED = 2

    AT_POINT_MARGIN = 0.05
    
    def __init__(self, name):
        self.name = name
        self.state = Odometry()
        rospy.Subscriber('/state', Odometry, self._receive_state)
        self._desired_state_pub = rospy.Publisher('/motion_planning/desired_state_global', PoseStamped, queue_size=10)

    def _receive_state(self, msg):
        self.state = msg
    
    def run(self):
        raise NotImplementedError('run method of task not implemented, or running base class')

    def pre_run(self):
        self.time_start = rospy.Time.now()

    def move_to_point(self, x, y, z):
        desired_pose = PoseStamped()
        desired_pose.pose.orientation = self.state.pose.pose.orientation

        if self.dist_from_self(x, y, z) < self.AT_POINT_MARGIN:
            self._desired_state_pub.publish(desired_pose)
            return True

        desired_pose.pose.position.x = x
        desired_pose.pose.position.y = y
        desired_pose.pose.position.z = z
        self._desired_state_pub.publish(desired_pose)
        return False

    def dist_from_self(self, x, y, z):
        self_point = np.array([self.state.pose.pose.position.x, self.state.pose.pose.position.y, self.state.pose.pose.position.z])
        other_point = np.array([x, y, z])
        return np.linalg.norm(other_point - self_point)


class TaskResult(Enum):
    CONTINUE = 1
    FINISHED = 2
