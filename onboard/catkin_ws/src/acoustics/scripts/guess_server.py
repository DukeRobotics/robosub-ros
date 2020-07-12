#!/usr/bin/env python

import rospy
import actionlib
from custom_msgs.msg import AcousticsGuessFeedback, AcousticsGuessResult, AcousticsGuessAction
from cheap_cross_cor import AcousticGuess
import sys
import os


class GuessServer:

    NODE_NAME = "acoustics_guesser"
    ACTION_NAME = "guess_acoustics"

    def __init__(self):
        rospy.init_node(self.NODE_NAME)
        self.server = actionlib.SimpleActionServer(self.ACTION_NAME, AcousticsGuessAction, self.execute, False)
        self.server.start()
        rospy.spin()

    def publish_feedback(self, curr_stage, total_stage):
        feedback = AcousticsGuessFeedback()
        feedback.curr_stage = curr_stage
        feedback.total_stage = total_stage
        self.server.publish_feedback(feedback)

    def publish_result(self, point):
        result = AcousticsGuessResult()
        result.guess.x = point[0]
        result.guess.y = point[1]
        result.guess.z = point[2]
        self.server.set_succeeded(result)

    def execute(self, goal):
        filepath = os.path.join(sys.path[0], '../data', goal.filename)
        processor = AcousticGuess(filepath, goal.samp_f, goal.tar_f, self.publish_feedback)
        point = processor.run()
        self.publish_result(point)


if __name__ == '__main__':
    GuessServer()
