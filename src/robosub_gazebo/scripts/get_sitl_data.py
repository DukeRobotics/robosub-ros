#!/usr/bin/env python

import rospy

from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import PoseStamped, Twist

class SitlToGazebo:

    NODE_NAME = 'sitl_to_gazebo'
    MODEL_NAME = 'robosub'
    REFERENCE_FRAME = 'world'
    PUB_RATE = 20

    GAZEBO_PUB_TOPIC = 'gazebo/set_model_state'
    SITL_POSE_TOPIC = 'mavros/local_position/pose'

    def __init__(self):
        self._last_sitl_pose_stamped = PoseStamped()
        self._sitl_pose_received = False

        self._pub = rospy.Publisher(self.GAZEBO_PUB_TOPIC, ModelState, queue_size=10)
        rospy.Subscriber(self.SITL_POSE_TOPIC, PoseStamped, self._receive_sitl_pose)

    def _receive_sitl_pose(self, pose):
        self._last_sitl_pose_stamped = pose
        self._sitl_pose_received = True

    def run(self):
        rospy.init_node(self.NODE_NAME)
        rate = rospy.Rate(self.PUB_RATE)

        while not rospy.is_shutdown():
            if not self._sitl_pose_received:
                continue

            model_state = ModelState()
            model_state.model_name = self.MODEL_NAME
            model_state.pose = self._last_sitl_pose_stamped.pose
            model_state.twist = Twist()
            model_state.reference_frame = self.REFERENCE_FRAME

            self._pub.publish(model_state)

if __name__ == '__main__':
    SitlToGazebo().run()
