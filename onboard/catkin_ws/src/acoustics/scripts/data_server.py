#!/usr/bin/env python

import rospy
import actionlib
from custom_msgs.msg import AcousticsDataFeedback, AcousticsDataResult, AcousticsDataAction
from data_sim import DataGenerator
import sys
import os


class DataServer:
    NODE_NAME = "data_generator"
    ACTION_NAME = "generate_data"

    def __init__(self):
        rospy.init_node(self.NODE_NAME)
        self.server = actionlib.SimpleActionServer(self.ACTION_NAME, AcousticsDataAction, self.execute, False)
        self.server.start()
        rospy.spin()

    def publish_result(self, success):  # success in generating data (bool)
        result = AcousticsDataResult()
        result.success = success
        self.server.set_succeeded(result)

    def publish_feedback(self, curr_file, total_file):  # files saved
        feedback = AcousticsDataFeedback()
        feedback.curr_file = curr_file
        feedback.total_file = total_file
        self.server.publish_feedback(feedback)

    def execute(self, goal):
        processor = DataGenerator(goal.samp_f, goal.tar_f, goal.hydrophone,
                                  [goal.plocation.x, goal.plocation.y, goal.plocation.z], self.publish_feedback)
        success = processor.run()
        self.publish_result(success)


if __name__ == '__main__':
    DataServer()
