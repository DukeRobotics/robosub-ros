#!/usr/bin/env python

import rospy
import actionlib
from custom_msgs.msg import AcousticsDataFeedback, AcousticsDataResult, AcousticsDataAction
from data_sim import DataGenerator


class DataServer:
    NODE_NAME = "acoustics_data_generator"
    ACTION_NAME = "generate_data"

    def __init__(self):
        rospy.init_node(self.NODE_NAME)
        self.server = actionlib.SimpleActionServer(self.ACTION_NAME, AcousticsDataAction, self.execute, False)
        self.server.start()
        rospy.spin()

    def publish_result(self, file_paths):
        result = AcousticsDataResult()
        result.file_paths = file_paths
        self.server.set_succeeded(result)

    def publish_feedback(self, curr_stage, total_stages, msg):  # files saved
        feedback = AcousticsDataFeedback()
        feedback.curr_stage = curr_stage
        feedback.total_stages = total_stages
        feedback.message = msg
        self.server.publish_feedback(feedback)

    def execute(self, goal):
        pinger_loc = (goal.location.x, goal.location.y, goal.location.z)
        file_paths = DataGenerator(goal.samp_f, goal.tar_f, goal.hydrophone_set, pinger_loc, self.publish_feedback).run()
        self.publish_result(file_paths)


if __name__ == '__main__':
    DataServer()
