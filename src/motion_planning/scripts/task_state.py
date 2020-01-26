#!/usr/bin/env python

import rospy

class TaskState(object):
    STATE_TOPIC = 'state'
    DESIRED_POSE_TOPIC = 'controls/desired_pose_global'
    DESIRED_TWIST_GLOBAL_TOPIC = 'controls/desired_twist_global'
    DESIRED_TWIST_LOCAL_TOPIC = 'controls/desired_twist_local'

    # Initialise static members that will hold the publishers and listeners
    STATE_LISTENER = None
    DESIRED_POSE_GLOBAL_PUBLISHER = None
    DESIRED_TWIST_GLOBAL_PUBLISHER = None
    DESIRED_TWIST_LOCAL_PUBLISHER = None
    STATE = None

    def __init__(self):
        self.STATE_LISTENER = rospy.Subscriber(self.STATE_TOPIC, Odometry, self._on_receive_state)
        self.DESIRED_POSE_GLOBAL_PUBLISHER = rospy.Publisher(self.DESIRED_POSE_TOPIC, Pose, queue_size=5)
        self.DESIRED_TWIST_GLOBAL_PUBLISHER = rospy.Publisher(self.DESIRED_TWIST_GLOBAL_TOPIC, Twist, queue_size=5)
        self.DESIRED_TWIST_LOCAL_PUBLISHER = rospy.Publisher(self.DESIRED_TWIST_LOCAL_TOPIC, Twist, queue_size=5)

    def _on_receive_state(self, state):
        """Receive the state, update initial_state if it is empty
        and the task is running"""
        self.STATE = state
        if self.initial_state is None and not self.started:
            self.initial_state = STATE