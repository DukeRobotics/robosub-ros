import rospy
import os
import yaml
from nav_msgs.msg import Odometry

class StateInterface:
    STATE_TOPIC = 'state'

    def __init__(self, tfBuffer):
        self.tfBuffer = tfBuffer
        rospy.Subscriber(self.STATE_TOPIC, Odometry, self._on_receive_state) 
        self._state = None

    @property
    def state(self):
        return self._state

    def _on_receive_state(self, state):
        self._state = state
        
    def get_state(self):
        return self._state