#!/usr/bin/env python

import rospy
import actionlib
from custom_msgs.msg import AcousticsProcessingFeedback, AcousticsProcessingResult, AcousticsProcessingAction
from cross_corr_fft import AcousticProcessor


class ProcessingServer:

    NODE_NAME = "acoustics_processor"
    ACTION_NAME = "process_acoustics"

    def __init__(self):
        rospy.init_node(self.NODE_NAME)
        self.server = actionlib.SimpleActionServer(self.ACTION_NAME, AcousticsProcessingAction, self.execute, False)
        self.server.start()
        self.total_count = 0
        rospy.spin()

    def publish_feedback(self, success_count, process_count):
        feedback = AcousticsProcessingFeedback()
        feedback.process_count = process_count
        feedback.success_count = success_count
        feedback.total_count = self.total_count
        self.server.publish_feedback(feedback)

    def publish_result(self, final_ccwh, valid_count, success_count):
        result = AcousticsProcessingResult()
        result.hz_angle = final_ccwh
        result.valid_count = valid_count
        result.success_count = success_count
        result.total_count = self.total_count
        self.server.set_succeeded(result)

    def execute(self, goal):
        self.total_count = len(goal.file_paths) * (2 if goal.if_double else 1)
        guess = (goal.guess.x, goal.guess.y, goal.guess.z)
        processor = AcousticProcessor(goal.file_paths, goal.if_double,
                                      goal.samp_f, goal.tar_f, guess, self.publish_feedback)
        final_ccwh, valid_count, success_count = processor.run()
        self.publish_result(final_ccwh, valid_count, success_count)


if __name__ == '__main__':
    ProcessingServer()
