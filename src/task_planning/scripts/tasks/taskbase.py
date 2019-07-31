#!/usr/bin/env python

from enum import Enum
from nav_msgs.msg import Odometry
import rospy

class TaskBase(object):

    CONTINUE = 1
    FINISHED = 2
    
    def __init__(self, name):
        self.name = name
        self.state = Odometry()
        rospy.Subscriber('/state', Odometry, self._receive_state)

    def _receive_state(self, msg):
        self.state = msg
    
    def run(self):
        raise NotImplementedError('Run method of task not implemented, or running base class')

class TaskResult(Enum):
    CONTINUE = 1
    FINISHED = 2
