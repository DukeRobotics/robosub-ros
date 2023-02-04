#!/usr/bin/env python3

import roslib
import rospy
import actionlib
from sonar import Sonar
from custom_msgs.msg import sweepAction, sweepResult
roslib.load_manifest('sonar')


""" The  following messages are created by the .action file
    sweepAction.msg
    sweepActionGoal.msg
    sweepActionResult.msg
    sweepActionFeedback.msg

    sweepGoal.msg                   #the parameters passed from the client
        float64 start_angle
        float64 end_angle
        float64 distance_of_scan
    sweepResult.msg                 #whats returned from to the client
        float64 x_pos
        float64 y_pos
    sweepFeedback.msg               #sent as continuous feedback to the robot (not implimented)
        int32 current_angle
"""


class SonarServer:

    _result = sweepResult()
    NODE_NAME = "sonar_server"
    ACTION_NAME = "sonar_sweep"

    def __init__(self):
        """ Creates action server with the sweepAction

        Callback: Runs the execute method

        """
        # 5m range
        self._sonar = Sonar()

        # rospy.init_node(self.NODE_NAME)
        self._server = actionlib.SimpleActionServer(self.ACTION_NAME, sweepAction, self.execute, auto_start=False)
        self._server.start()
        rospy.spin()

    def execute(self, goal):
        self._sonar.set_new_range(goal.distance_of_scan)
        pos_tuple = self._sonar.get_xy_of_object_in_sweep(goal.start_angle, goal.end_angle)

        # Sends everything as a result to the client
        self._result.x_pos = pos_tuple[0]
        self._result.y_pos = pos_tuple[1]
        self._server.set_succeeded(self._result)


if __name__ == '__main__':
    SonarServer()
