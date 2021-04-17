#!/usr/bin/env python

import shlex
import rospy
import actionlib
import subprocess
import shutil
import os
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
        current_dir = os.getcwd()
        print(current_dir)
        subprocess.check_call("cd ~/dev/robosub-ros/onboard/catkin_ws/src/system_utils/scripts", shell=True)
        ##subprocess.check_call("export PATH=\"/root/dev/robosub-ros/test:$PATH\"")
        subprocess.check_call("rosrun rosserial_arduino make_libraries.py ~/dev/robosub-ros/onboard/catkin_ws/src/system_utils/scripts", shell=True)
        os.chdir('/root/dev/robosub-ros/onboard/catkin_ws/src/system_utils/scripts')
        current_dir = os.getcwd()
        print(current_dir)
        shutil.make_archive('/root/dev/robosub-ros/onboard/catkin_ws/src/system_utils/scripts/ros_lib', 'zip', '/root/dev/robosub-ros/onboard/catkin_ws/src/system_utils/scripts', 'ros_lib')
        subprocess.check_call('arduino-cli') #assuming arduino cli is part of path variables
        ##subprocess.check_call("export ARDUINO_LIBRARY_ENABLE_UNSAFE_INSTALL=true", shell=True)
        subprocess.check_call("arduino-cli lib install --zip-path /root/dev/robosub-ros/onboard/catkin_ws/src/system_utils/scripts/ros_lib.zip", shell = True)
        subprocess.check_call("rm ros_lib.zip", shell = True)
        subprocess.check_call("arduino-cli core install arduino:avr", shell = True) #for the robot use arduino:samd to download the arduino nano core
        subprocess.check_call("arduino-cli compile -b arduino:avr:nano ../data/" + goal.filename, shell = True)
        subprocess.check_call("arduino-cli upload -b arduino:avr:mega -p " + goal.port + " ../data/" + goal.filename, shell = True)
        self.publish_result()



        



if __name__ == '__main__':
    Saleae()