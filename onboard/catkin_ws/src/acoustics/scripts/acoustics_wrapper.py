#!/usr/bin/env python

import rospy
import actionlib
from custom_msgs.msg import AcousticsGuessGoal, AcousticsGuessAction, AcousticsProcessingGoal, AcousticsProcessingAction, AcousticsWrapperMessageAction, AcousticsWrapperMessageFeedback, AcousticsWrapperMessageResult
import sys
import os


class ServerClientThing:

    NODE_NAME = "acoustics_wrapper"
    ACTION_NAME = "call_guess_processing"

    def __init__(self):
        rospy.init_node(self.NODE_NAME)
        self.client_guess = actionlib.SimpleActionClient('guess_acoustics', AcousticsGuessAction)  # name might actually be action name
        self.client_processing = actionlib.SimpleActionClient('process_acoustics', AcousticsProcessingAction)
        self.server = actionlib.SimpleActionServer(self.ACTION_NAME, AcousticsWrapperMessageAction, self.execute, False)
        self.server.start()
        rospy.spin()


    def publish_feedback(self, stage):
        feedback = AcousticsWrapperMessageFeedback()
        feedback.curr_stage = stage;
        feedback.total_stage = 2
        self.server.publish_feedback(feedback)

    def publish_result(self, processing_result):
        result = AcousticsWrapperMessageResult()
        result.hz_angle = processing_result.hz_angle
        self.server.set_succeeded(result)

    def execute(self, this_goal):
        self.client_guess.wait_for_server()
        goal_client_guess = AcousticsGuessGoal()
        goal_client_guess.filename = this_goal.filename
        goal_client_guess.samp_f = 625000
        goal_client_guess.tar_f = this_goal.tar_f
        self.client_guess.send_goal(goal_client_guess)
        self.client_guess.wait_for_result()
        guess = self.client_guess.get_result().guess
        self.publish_feedback(1)

        self.client_processing.wait_for_server()
        goal_client_processing = AcousticsProcessingGoal()
        goal_client_processing.filename = this_goal.filename
        goal_client_processing.if_double = True
        goal_client_processing.version = 0
        goal_client_processing.samp_f = 625000
        goal_client_processing.tar_f = this_goal.tar_f
        goal_client_processing.guess = guess;
        self.client_processing.send_goal(goal_client_processing)
        self.client_processing.wait_for_result()
        self.publish_feedback(2)

        result = self.client_processing.get_result()
        self.publish_result(result)


if __name__ == '__main__':
    ServerClientThing()