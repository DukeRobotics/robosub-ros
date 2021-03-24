#!/usr/bin/env python

import rospy
import actionlib
import subprocess
from custom_msgs.action import UploadArduinoAction, UploadArduinoFeedback, UploadArduinoResult
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

    def publish_result(self):
        result = UploadArduinoResult(success=True)
        self.server.set_succeeded(result)

    def execute(self, goal):
        subprocess.check_call("arduino-cli") #assuming arduino cli is part of path variables
        subprocess.check_call("arduino-cli core install arduino:avd") #for the robot use arduino:samd to download the arduino nano core
        subprocess.check_call(["arduino-cli compile -b arduino:avd:mega ..\data\\", goal.filename])
        subprocess.check_call(["arduino-cli upload -b arduino:avd:mega -p ", goal.port, " ..\data\\", goal.filename])
        self.publish_result()


if __name__ == '__main__':
    Saleae()
