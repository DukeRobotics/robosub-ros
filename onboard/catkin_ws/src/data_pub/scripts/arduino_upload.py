#!/usr/bin/env python

import rospy
import actionlib
from custom_msgs.msg import UploadArduinoAction, UploadArduinoFeedback, UploadArduinoResult
import resource_retriever as rr


class Saleae:

    NODE_NAME = "arduino_upload"
    ACTION_NAME = "upload_arduino"

    def __init__(self):
        rospy.init_node(self.NODE_NAME)
        self.server = actionlib.SimpleActionServer(self.ACTION_NAME, UploadArduinoAction, self.execute, False)
        self.server.start()
        rospy.spin()

    def publish_feedback(self, stage, total_stages, msg):
        feedback = UploadArduinoFeedback()
        feedback.curr_stage = stage
        feedback.total_stages = total_stages
        feedback.message = msg
        self.server.publish_feedback(feedback)

    def publish_result(self, save_paths):
        result = UploadArduinoResult(file_paths=save_paths)
        self.server.set_succeeded(result)

    def execute(self, goal):
        



if __name__ == '__main__':
    Saleae()