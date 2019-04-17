#!/usr/bin/env python

import rospy

from controls.msg import DesiredStateLocal

class MoveToLocalPose:

    NODE_NAME = 'local_pose_movement'
    LISTENING_TOPIC = 'desired_pose/local'

    def __init__(self):
        self._desired_local_state = DesiredStateLocal()
        self._desired_relative_speeds = [0] * 6

    def _on_receive(self, msg):
        self._desired_local_state = msg.state
        self._desired_relative_speeds = msg.speeds

    def run(self):
        rospy.init_node(self.NODE_NAME)

        rospy.Subscriber(LISTENING_TOPIC, DesiredStateLocal, _on_receive)

if __name__ == '__main__':
    MoveToLocalPose().run()
