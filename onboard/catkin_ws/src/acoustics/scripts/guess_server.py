#!/usr/bin/env python

import rospy
import actionlib
from custom_msgs.msg import AcousticsGuessFeedback, AcousticsGuessResult, AcousticsGuessAction
from cheap_cross_cor import AcousticGuess


class GuessServer:

    NODE_NAME = "acoustics_guess"
    ACTION_NAME = "guess_acoustics"

    def __init__(self):
        rospy.init_node(self.NODE_NAME)
        self.server = actionlib.SimpleActionServer(self.ACTION_NAME, AcousticsGuessAction, self.execute, False)
        self.server.start()
        rospy.spin()

    def publish_feedback(self, filtered, cross_correlated):
        feedback = AcousticsGuessFeedback()
        feedback.filtered = filtered
        feedback.cross_correlated = cross_correlated
        self.server.publish_feedback(feedback)

    def publish_result(self, point):
        result = AcousticsGuessResult()
        result.guess.x = float(point[0])
        result.guess.y = float(point[1])
        result.guess.z = float(point[2])
        self.server.set_succeeded(result)

    def execute(self, goal):
        processor = AcousticGuess(goal.filename, goal.samp_f, goal.tar_f, self.publish_feedback)
        point = processor.run()
        self.publish_result(point)


if __name__ == '__main__':
    GuessServer()
