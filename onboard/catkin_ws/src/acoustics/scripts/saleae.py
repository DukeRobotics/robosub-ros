#!/usr/bin/env python

import rospy
import actionlib
from custom_msgs.msg import SaleaeAction, SaleaeFeedback, SaleaeResult, HydrophoneSet
import pandas as pd
import saleae
import resource_retriever as rr
from datetime import datetime


class Saleae:

    NODE_NAME = "saleae"
    ACTION_NAME = "call_saleae"
    IP_ADDRESS = "localhost"
    PORT = 10429
    CAPTURE_DURATION = 2
    HYDROPHONE_SET = {HydrophoneSet.GUESS: [0, 1, 2, 3], HydrophoneSet.PROCESS: [4, 5, 6, 7]}
    FILE_EXTENSIONS = {HydrophoneSet.GUESS: "_guess", HydrophoneSet.PROCESS: "_processing"}

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
        df = pd.read_csv(file)
        df.drop([0])
        df.drop([0], axis=1)
        df.to_csv(file, index=False)

    def execute(self, goal):
        self.saleae.set_capture_seconds(goal.capture_duration)
        
        time_now = datetime.now()
        export_name = "date_" + time_now.strftime("%m_%d_%Y_%H_%M_%S") + self.FILE_EXTENSIONS[goal.hydrophone_set.type]
        package_path = 'package://acoustics/data/' + export_name + '_({1}).csv'
        save_paths = []

        for i in range(goal.capture_count):
            self.publish_feedback(i + 1, goal.capture_count + 1, "Starting capture {}".format(i))
            save_paths.append(package_path.format(i))
            save_path = rr.get_filename(package_path.format(i), use_protocol=False)
            self.saleae.export_data2(save_path, analog_channels=self.HYDROPHONE_SET[goal.hydrophone_set.type])
            self.format_csv(save_path)

        self.publish_feedback(goal.capture_count + 1, goal.capture_count + 1, "Saleae capture complete")
        self.publish_result(save_paths)


if __name__ == '__main__':
    Saleae()
