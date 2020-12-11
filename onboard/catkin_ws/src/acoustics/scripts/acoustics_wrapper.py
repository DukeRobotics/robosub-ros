#!/usr/bin/env python

import rospy
import actionlib
from custom_msgs.msg import AcousticsGuessGoal, AcousticsGuessAction, \
    AcousticsProcessingGoal, AcousticsProcessingAction, \
    SaeleaeGoal, SaeleaeAction, \
    AcousticsWrapperAction, AcousticsWrapperFeedback, AcousticsWrapperResult
from datetime import datetime
import sys
import os



class AcousticsWrapper:

    NODE_NAME = "acoustics_wrapper"
    ACTION_NAME = "call_guess_processing"

    GUESS_SAMPLE_FREQ = 625000
    PROCESSING_SAMPLE_FREQ = 625000
    IF_DOUBLE = True
    PROCESSING_VERSION = 0


    def __init__(self):
        rospy.init_node(self.NODE_NAME)
        #
        self.client_sampling = actionlib.SimpleActionClient('call_saeleae', SaeleaeAction)
        #
        self.client_guess = actionlib.SimpleActionClient('guess_acoustics', AcousticsGuessAction)
        self.client_processing = actionlib.SimpleActionClient('process_acoustics', AcousticsProcessingAction)
        self.server = actionlib.SimpleActionServer(self.ACTION_NAME, AcousticsWrapperAction, self.execute, False)
        self.server.start()
        rospy.spin()

    def publish_feedback(self, stage):
        feedback = AcousticsWrapperFeedback()
        feedback.curr_stage = stage;
        feedback.total_stage = 2
        self.server.publish_feedback(feedback)

    def publish_result(self, processing_result):
        result = AcousticsWrapperResult()
        result.hz_angle = processing_result.hz_angle
        self.server.set_succeeded(result)

    def create_sample_file(self, file_use, date):
        time_file_name = date.strftime("%m_%d_%Y; %H.%M.%S") + "; " + file_use + ".csv"
        filepath = os.path.join(sys.path[0], '../data', time_file_name)
        with open(filepath, 'w') as fp:
            pass
        fp.close()
        return time_file_name

    def get_guess(self, this_goal, guess_file):
        self.client_guess.wait_for_server()
        goal_client_guess = AcousticsGuessGoal()
        goal_client_guess.filename = guess_file
        goal_client_guess.samp_f = self.GUESS_SAMPLE_FREQ
        goal_client_guess.tar_f = this_goal.tar_f
        self.client_guess.send_goal(goal_client_guess)
        self.client_guess.wait_for_result()
        return self.client_guess.get_result().guess

    def get_processing_angle(self, this_goal, processing_file, guess):
        self.client_processing.wait_for_server()
        goal_client_processing = AcousticsProcessingGoal()
        goal_client_processing.filename = processing_file
        goal_client_processing.if_double = self.IF_DOUBLE
        goal_client_processing.version = self.PROCESSING_VERSION
        goal_client_processing.samp_f = self.PROCESSING_SAMPLE_FREQ
        goal_client_processing.tar_f = this_goal.tar_f
        goal_client_processing.guess = guess
        self.client_processing.send_goal(goal_client_processing)
        self.client_processing.wait_for_result()
        return self.client_processing.get_result()

    def execute(self, this_goal):
        time_now = datetime.now()
        guess_data = self.create_sample_file("guess", time_now)
        processing_data = self.create_sample_file("processing", time_now)

        #
        self.client_sampling.wait_for_server()
        goal_client_sampling = SaeleaeGoal()
        goal_client_sampling.hydrophone_set = 1
        goal_client_sampling.save_path = guess_data
        goal_client_sampling.capture_directory = ''
        self.client_sampling.send_goal(goal_client_sampling)
        self.client_sampling.wait_for_result()
        guess_file_name = self.client_sampling.get_result().return_file_name
        self.publish_feedback(1)
        #

        guess_file = 'matlab_custom_cheap_hydrophone_-2_7_4_(1).csv'
        guess = self.get_guess(this_goal, guess_file)
        self.publish_feedback(2)

        #
        self.client_sampling.wait_for_server()
        goal_client_sampling = SaeleaeGoal()
        goal_client_sampling.hydrophone_set = 2
        goal_client_sampling.save_path = processing_data
        goal_client_sampling.capture_directory = ''
        self.client_sampling.send_goal(goal_client_sampling)
        self.client_sampling.wait_for_result()
        processing_file_name = self.client_sampling.get_result().return_file_name
        self.publish_feedback(1)
        #

        processing_file = 'matlab_custom_cheap_hydrophone_-2_7_4_(1).csv'
        processing_result = self.get_processing_angle(this_goal, processing_file, guess)
        self.publish_feedback(2)

        self.publish_result(processing_result)



if __name__ == '__main__':
    AcousticsWrapper()