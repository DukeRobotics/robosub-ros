#!/usr/bin/env python

import rospy
import actionlib
from custom_msgs.msg import AcousticsGuessGoal, AcousticsGuessAction, \
    AcousticsProcessingGoal, AcousticsProcessingAction, \
    SaleaeGoal, SaleaeAction, \
    AcousticsDataGoal, AcousticsDataAction, \
    AcousticsWrapperAction, AcousticsWrapperFeedback, AcousticsWrapperResult, \
    HydrophoneSet


class AcousticsWrapper:

    NODE_NAME = "acoustics_wrapper"
    ACTION_NAME = "call_guess_processing"

    SAMPLING_FREQ = {HydrophoneSet.GUESS: 625000, HydrophoneSet.PROCESS: 625000}
    CAPTURE_COUNT = 4
    CAPTURE_DURATION = 2

    def __init__(self):
        rospy.init_node(self.NODE_NAME)
        if rospy.get_param('~sim'):
            self.client_sampling = actionlib.SimpleActionClient('generate_data', AcousticsDataAction)
            self.server = actionlib.SimpleActionServer(
                self.ACTION_NAME, AcousticsWrapperAction, lambda goal: self.execute(
                    goal, self.generate_data), False)
        else:
            self.client_sampling = actionlib.SimpleActionClient('call_saleae', SaleaeAction)
            self.server = actionlib.SimpleActionServer(
                self.ACTION_NAME, AcousticsWrapperAction, lambda goal: self.execute(
                    goal, self.saleae_sampling), False)
        self.client_guess = actionlib.SimpleActionClient('guess_acoustics', AcousticsGuessAction)
        self.client_processing = actionlib.SimpleActionClient('process_acoustics', AcousticsProcessingAction)

        self.client_sampling.wait_for_server()
        self.client_guess.wait_for_server()
        self.client_processing.wait_for_server()

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

    def saleae_sampling(self, wrapper_goal, hydrophone_set):
        goal = SaleaeGoal()
        goal.hydrophone_set.type = hydrophone_set
        goal.capture_count = self.CAPTURE_COUNT
        goal.capture_duration = self.CAPTURE_DURATION
        self.client_sampling.send_goal(goal)
        self.client_sampling.wait_for_result()
        return self.client_processing.get_result().file_paths

    def generate_data(self, wrapper_goal, hydrophone_set):
        goal = AcousticsDataGoal()
        goal.hydrophone_set.type = hydrophone_set
        goal.samp_f = self.SAMPLING_FREQ[hydrophone_set]
        goal.tar_f = wrapper_goal.tar_f
        goal.location = wrapper_goal.location
        self.client_sampling.send_goal(goal)
        self.client_sampling.wait_for_result()
        return self.client_sampling.get_result().file_paths

    def get_guess(self, wrapper_goal, guess_files):
        goal = AcousticsGuessGoal()
        goal.file_paths = guess_files
        goal.samp_f = self.SAMPLING_FREQ[HydrophoneSet.GUESS]
        goal.tar_f = wrapper_goal.tar_f
        self.client_guess.send_goal(goal)
        self.client_guess.wait_for_result()
        return self.client_guess.get_result().guess

    def get_processing_angle(self, wrapper_goal, processing_files, guess):
        goal = AcousticsProcessingGoal()
        goal.file_paths = processing_files
        goal.samp_f = self.SAMPLING_FREQ[HydrophoneSet.PROCESS]
        goal.tar_f = wrapper_goal.tar_f
        goal.guess = guess
        self.client_processing.send_goal(goal)
        self.client_processing.wait_for_result()
        return self.client_processing.get_result().hz_angle

    def execute(self, wrapper_goal, sampling_fn):

        self.publish_feedback(0, "Starting sampling for guess")
        guess_filepaths = sampling_fn(wrapper_goal, HydrophoneSet.GUESS)
        self.publish_feedback(1, "Generating data complete, starting guess")

        guess = self.get_guess(wrapper_goal, guess_filepaths)
        self.publish_feedback(2, "Guess complete, generating data for processing")

        processing_filepaths = sampling_fn(wrapper_goal, HydrophoneSet.PROCESS)
        self.publish_feedback(3, "Generating data for processing complete, starting processing")

        processing_result = self.get_processing_angle(wrapper_goal, processing_filepaths, guess)
        self.publish_feedback(4, "Processing complete")

        self.publish_result(processing_result)


if __name__ == '__main__':
    AcousticsWrapper()
