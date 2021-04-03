#!/usr/bin/env python

import rospy
import actionlib
import subprocess
from custom_msgs.msg import UploadArduinoAction, UploadArduinoFeedback, UploadArduinoResult

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
        subprocess.check_call("export PATH=\"~/dev/robosub-ros/test:$PATH\"")
        subprocess.check_call("rosrun rosserial_arduino make_libraries.py .")
        subprocess.check_call("zip ros_lib.zip ros_lib")
        subprocess.check_call("arduino-cli") #assuming arduino cli is part of path variables
        subprocess.check_call("export ARDUINO_LIBRARY_ENABLE_UNSAFE_INSTALL=true")
        subprocess.check_call("arduino-cli lib install --zip-path ros_lib.zip")
        subprocess.check_call("rm -r ros_lib")
        subprocess.check_call("rm ros_lib.zip")
        subprocess.check_call("arduino-cli core install arduino:avr") #for the robot use arduino:samd to download the arduino nano core
        subprocess.check_call("arduino-cli compile -b arduino:avr:mega ../data/" + goal.filename)
        subprocess.check_call("arduino-cli upload -b arduino:avr:mega -p " + goal.port + " ../data/" + goal.filename)
        self.publish_result()



        



if __name__ == '__main__':
    Saleae()