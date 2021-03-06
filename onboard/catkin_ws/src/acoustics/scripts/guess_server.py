#!/usr/bin/env python

import rospy
import actionlib
from custom_msgs.msg import AcousticsGuessFeedback, AcousticsGuessResult, AcousticsGuessAction
from cheap_cross_cor import AcousticGuess


class GuessServer:

    NODE_NAME = "acoustics_guesser"
    ACTION_NAME = "guess_acoustics"

    def __init__(self):
        rospy.init_node(self.NODE_NAME)
        self.server = actionlib.SimpleActionServer(self.ACTION_NAME, AcousticsGuessAction, self.execute, False)
        self.server.start()
        rospy.spin()

    def publish_feedback(self, curr_stage, total_stages, msg):
        feedback = AcousticsGuessFeedback()
        feedback.curr_stage = curr_stage
        feedback.total_stages = total_stages
        feedback.message = msg
        self.server.publish_feedback(feedback)

    def publish_result(self, point):
        result = AcousticsGuessResult()
        result.guess.x = point[0]
        result.guess.y = point[1]
        result.guess.z = point[2]
        self.server.set_succeeded(result)

    def execute(self, goal):
        point = AcousticGuess(goal.file_paths, goal.samp_f, goal.tar_f, self.publish_feedback).run()
        self.publish_result(point)


if __name__ == '__main__':
    GuessServer()
