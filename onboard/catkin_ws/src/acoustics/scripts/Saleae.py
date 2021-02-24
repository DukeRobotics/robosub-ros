#!/usr/bin/env python

import rospy
import actionlib
from custom_msgs.msg import AcousticsGuessGoal, AcousticsGuessAction, \
    AcousticsProcessingGoal, AcousticsProcessingAction, \
    SaleaeAction, SaleaeFeedback, SaleaeResult, SaleaeGoal
import saleae
import sys
import os
import resource_retriever as rr



class Saleae:

    NODE_NAME = "saleae"
    ACTION_NAME = "call_saleae"
    IP_ADDRESS = 'localhost'
    PORT = 10429
    CAPTURE_COUNT = 1
    CAPTURE_DURATION = 1
    HYDROPHONE_SET_1 = [0, 3]
    HYDROPHONE_SET_2 = [4, 7]

    def __init__(self):
        rospy.init_node(self.NODE_NAME)
        self.server = actionlib.SimpleActionServer(self.ACTION_NAME, SaleaeAction, self.execute, False)
        self.server.start()
        rospy.spin()

    def publish_feedback(self, stage):
        feedback = SaleaeFeedback()
        feedback.curr_stage = stage
        feedback.total_stage = 2
        self.server.publish_feedback(feedback)

    def publish_result(self, return_file):
        result = SaleaeResult()
        result.return_file_name = return_file
        self.server.set_succeeded(result)

    def validate_path(self, path):
        if path is not None:
            if not os.path.dirname(path):
                rospy.logerr('the specified ' + ' directory does not exist or is invalid')
                rospy.logerr('you specified: ' + path)
                exit(1)

    def execute(self, goal):
        rospy.loginfo("a")
        s = saleae.Saleae(self.IP_ADDRESS, self.PORT)
        goal_name = goal.save_name
        file_name = "/root/dev/robosub-ros/onboard/catkin_ws/src/acoustics/data/" + goal_name + "{0}.{1}"
        # "/root/dev/robosub-ros/onboard/catkin_ws/src/acoustics/data/{0}.{1}"
        s.set_capture_seconds(self.CAPTURE_DURATION)
        save_path = ""
        self.publish_feedback(1)
        if goal.hydrophone_set == 1:
            for i in range(self.CAPTURE_COUNT):
               # s.capture_to_file(file_name.format(i, "logicdata"))
                save_path = file_name.format(i, "csv")
                print('exporting data to ' + save_path)
                s.export_data2(save_path, analog_channels=[0, 1, 2, 3])
        if goal.hydrophone_set == 2:
            for i in range(self.CAPTURE_COUNT):
               # s.capture_to_file(file_name.format(i, "logicdata"))
                save_path = file_name.format(i, "csv")
                print('exporting data to ' + save_path)
                s.export_data2(save_path, analog_channels=[4, 5, 6, 7])
        self.publish_feedback(2)
        self.publish_result(save_path)

if __name__ == '__main__':
    Saleae()

