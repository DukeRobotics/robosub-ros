#!/usr/bin/env python

import rospy

from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import PoseStamped

# Run sitl and look at what is published
# Make guide on how to run sitl
# Just get sitl data and republish it to /gazebo/set_model_state (gazebo_msgs/ModelState)
#  - Just use the pose

class SitlToGazebo:

    NODE_NAME = "sitl_to_gazebo"

    def __init__(self):
        self._last_sitl_state = PoseStamped()
        self._sitl_pose_received = False

        self._pub = rospy.Publisher(self.GAZEBO_PUB_TOPIC, ModelState queue_size=10)
        rospy.Subscriber(self.SITL_POSE_TOPIC, PoseStamped, self._receive_sitl_pose)

    def _receive_sitl_pose(self, pose):
        self._last_sitl_state = pose
        self._sitl_pose_received = True

    def run(self):
        rospy.init_node(self.NODE_NAME)
        rate = rospy.Rate(self.PUB_RATE)

        while not rospy.is_shutdown():
            if not self._sitl_pose_received:
                continue

            self._pub.Publish



    rospy.init_node(NODE_NAME)

sitl_
