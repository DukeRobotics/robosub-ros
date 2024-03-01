import rospy
import os
import yaml
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

class StateInterface:
    STATE_TOPIC = 'state'
    RESET_POSE_SERVICE = '/set_pose'


    def __init__(self, tfBuffer):

        self.tfBuffer = tfBuffer
        rospy.Subscriber(self.STATE_TOPIC, Odometry, self._on_receive_state) 
        self._state = None

        rospy.wait_for_service(self.RESET_POSE_SERVICE)
        self._reset_pose = rospy.ServiceProxy(self.RESET_POSE_SERVICE, PoseWithCovarianceStamped)


    @property
    def state(self):
        return self._state

    def _on_receive_state(self, state):
        self._state = state

    def get_state(self):
        return self._state
    
    def reset_pose(self):
        poseCov = PoseWithCovarianceStamped()
        poseCov.pose.pose.orientation.w = 1
        self._reset_pose(poseCov)
        
        return