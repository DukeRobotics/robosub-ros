#!/usr/bin/env python

import rospy
import actionlib
from custom_msgs.msg import AcousticsGuessGoal, AcousticsGuessAction, \
    AcousticsProcessingGoal, AcousticsProcessingAction, \
    SaleaeGoal, SaleaeAction, \
    AcousticsDataGoal, AcousticsDataAction, \
    AcousticsWrapperAction, AcousticsWrapperFeedback, AcousticsWrapperResult

from datetime import datetime


class AcousticsWrapper:

    NODE_NAME = "acoustics_wrapper"
    ACTION_NAME = "call_guess_processing"

    GUESS_SAMPLE_FREQ = 625000
    PROCESSING_SAMPLE_FREQ = 625000
    CAPTURE_COUNT = 4
    CAPTURE_DURATION = 2

    def __init__(self):
        rospy.init_node(self.NODE_NAME)
        self.client_sampling = actionlib.SimpleActionClient('call_saleae', SaleaeAction)
        self.client_guess = actionlib.SimpleActionClient('guess_acoustics', AcousticsGuessAction)
        self.client_processing = actionlib.SimpleActionClient('process_acoustics', AcousticsProcessingAction)
        self.client_data = actionlib.SimpleActionClient('generate_data', AcousticsDataAction)

        self.client_sampling.wait_for_server()
        self.client_guess.wait_for_server()
        self.client_processing.wait_for_server()
        self.client_data.wait_for_server()

        self.server = actionlib.SimpleActionServer(self.ACTION_NAME, AcousticsWrapperAction, self.execute, False)
        self.server.start()

        rospy.spin()

    def publish_feedback(self, stage, msg):
        feedback = AcousticsWrapperFeedback()
        feedback.curr_stage = stage
        feedback.total_stages = 4
        feedback.message = msg
        self.server.publish_feedback(feedback)

    def publish_result(self, hz_angle):
        result = AcousticsWrapperResult()
        result.hz_angle = hz_angle
        self.server.set_succeeded(result)

    def saleae_sampling(self, hydrophone_set, save_name):
        goal = SaleaeGoal()
        goal.hydrophone_set = hydrophone_set
        goal.save_name = save_name
        goal.capture_count = self.CAPTURE_COUNT
        goal.capture_duration = self.CAPTURE_DURATION
        self.client_sampling.send_goal(goal)
        self.client_sampling.wait_for_result()
        return self.client_processing.get_result()

    def generate_data(self, hydrophone_set, wrapper_goal):
        goal = AcousticsDataGoal()
        goal.hydrophone_set = hydrophone_set
        if hydrophone_set == 1:
            goal.samp_f = self.GUESS_SAMPLE_FREQ
        else:
            goal.samp_f = self.PROCESSING_SAMPLE_FREQ
        goal.tar_f = wrapper_goal.tar_f
        goal.location = wrapper_goal.location
        self.client_data.send_goal(goal)
        self.client_data.wait_for_result()
        return self.client_datae.get_result()


    def get_guess(self, wrapper_goal, guess_files):
        goal = AcousticsGuessGoal()
        goal.file_paths = guess_files
        goal.samp_f = self.GUESS_SAMPLE_FREQ
        goal.tar_f = wrapper_goal.tar_f
        self.client_guess.send_goal(goal)
        self.client_guess.wait_for_result()
        return self.client_guess.get_result().guess

    def get_processing_angle(self, wrapper_goal, processing_files, guess):
        goal = AcousticsProcessingGoal()
        goal.file_paths = processing_files
        goal.samp_f = self.PROCESSING_SAMPLE_FREQ
        goal.tar_f = wrapper_goal.tar_f
        goal.guess = guess
        self.client_processing.send_goal(goal)
        self.client_processing.wait_for_result()
        return self.client_processing.get_result()

    def execute(self, wrapper_goal):

        time_now = datetime.now()
        guess_export_name = "date_" + time_now.strftime("%m_%d_%Y_%H_%M_%S") + "_guess"
        processing_export_name = "date_" + time_now.strftime("%m_%d_%Y_%H_%M_%S") + "_processing"

        if wrapper_goal.gen_data == True:
            self.publish_feedback(0, "Generating data for guess")
            guess_sampling = self.generate_data(1, wrapper_goal)
            self.publish_feedback(1, "Generating data complete, starting guess")

            guess = self.get_guess(wrapper_goal, guess_sampling.file_paths)
            self.publish_feedback(2, "Guess complete, generating data for processing")

            processing_sampling = self.saleae_sampling(2, processing_export_name)
            self.publish_feedback(3, "Generating data for processing complete, starting processing")
        else:
            self.publish_feedback(0, "Starting sampling for guess")
            guess_sampling = self.saleae_sampling(1, guess_export_name)
            self.publish_feedback(1, "Sampling for guess complete, starting guess")

            guess = self.get_guess(wrapper_goal, guess_sampling.file_paths)
            self.publish_feedback(2, "Guess complete, starting sampling for processing")

            processing_sampling = self.saleae_sampling(2, processing_export_name)
            self.publish_feedback(3, "Sampling for processing complete, starting processing")

        processing_result = self.get_processing_angle(wrapper_goal, processing_sampling.file_paths, guess)
        self.publish_feedback(4, "Processing complete")

        self.publish_result(processing_result.hz_angle)


if __name__ == '__main__':
    AcousticsWrapper()
