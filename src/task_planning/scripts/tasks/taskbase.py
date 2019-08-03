#!/usr/bin/env python

from enum import Enum
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import rospy
import numpy as np
import tf2_ros

class TaskBase(object):

    CONTINUE = 1
    FINISHED = 2

    AT_POINT_MARGIN = 0.05
    
    def __init__(self, name):
        self.name = name
        self.state = Odometry()
        rospy.Subscriber('/state', Odometry, self._receive_state)
        self._desired_state_pub = rospy.Publisher('/motion_planning/desired_state_global', PoseStamped, queue_size=10)
        self._tfBuffer = tf2_ros.Buffer()
        self._tfListener = tf2_ros.TransformListener(self._tfBuffer)

    def _receive_state(self, msg):
        self.state = msg
    
    def run(self):
        raise NotImplementedError('run method of task not implemented, or running base class')

    def pre_run_base(self):
        self.time_start = rospy.Time.now()
        self.task_start_transform_to_global = self._tfBuffer.lookup_transform('odom', 'base_link', rospy.Time(0), rospy.Duration(0.5))

    def pre_run(self):
        pass

    def move_to_point(self, location):
        if self.dist_from_self(location.pose.position.x,
                               location.pose.position.y,
                               location.pose.position.z) < self.AT_POINT_MARGIN:
            return True

        self._desired_state_pub.publish(location)
        return False

    def dist_from_self(self, x, y, z):
        self_point = np.array([self.state.pose.pose.position.x, self.state.pose.pose.position.y, self.state.pose.pose.position.z])
        other_point = np.array([x, y, z])
        return np.linalg.norm(other_point - self_point)


class TaskResult(Enum):
    CONTINUE = 1
    FINISHED = 2
