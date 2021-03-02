#!/usr/bin/env python

import rospy
import actionlib
from custom_msgs.msg import SaleaeAction, SaleaeFeedback, SaleaeResult
import saleae
import sys
import os
import resource_retriever as rr


class Saleae:

    NODE_NAME = "saleae"
    ACTION_NAME = "call_saleae"
    IP_ADDRESS = "localhost"
    PORT = 10429
    CAPTURE_DURATION = 2
    HYDROPHONE_SET = {1: [0, 1, 2, 3], 2: [4, 5, 6, 7]}

    def __init__(self):
        rospy.init_node(self.NODE_NAME)
        self.server = actionlib.SimpleActionServer(self.ACTION_NAME, SaleaeAction, self.execute, False)
        self.server.start()
        self.saleae = saleae.Saleae(self.IP_ADDRESS, self.PORT)
        rospy.spin()

    def publish_feedback(self, stage, total_stages, msg):
        feedback = SaleaeFeedback()
        feedback.curr_stage = stage
        feedback.total_stages = total_stages
        feedback.message = msg
        self.server.publish_feedback(feedback)

    def publish_result(self, save_paths):
        result = SaleaeResult(file_paths=save_paths)
        self.server.set_succeeded(result)

    def format_csv(self, file):
        df = pd.read_csv(abs_path)
        df.drop([0])
        df.drop([0], axis=1)
        df.to_csv(abs_path, index=False)

    def execute(self, goal):
        self.saleae.set_capture_seconds(goal.capture_duration)
        package_path = 'package://acoustics/data/' + goal.save_name + '_{1}.csv'
        save_paths = []

        for i in range(goal.capture_count):
            self.publish_feedback(i + 1, goal.capture_count + 1, "Starting capture {}".format(i))
            save_paths.append(package_path.format(i))
            save_path = rr.get_filename(package_path.format(i), use_protocol=False)
            self.saleae.export_data2(save_path, analog_channels=self.HYDROPHONE_SET[goal.hydrophone_set])
            self.format_csv(save_path)

        self.publish_feedback(goal.capture_count + 1, goal.capture_count + 1, "Saleae capture complete")
        self.publish_result(save_paths)

if __name__ == '__main__':
    Saleae()
