#!/usr/bin/env python

import os
import sys
import rospy
import rosbag
from cv_bridge import CvBridge


class BagVideoConverter:

    def __init__(self):
        rospy.init_node("bag_video_converter", anonymous=True)
        self.bag_file = os.path.join(sys.path[0], "../bag", rospy.get_param("~bag_file"))
        video_file_list = [os.path.join(sys.path[0], "../avi", file) for file in rospy.get_param("~video_file")]
        self.topic_name = rospy.get_param("~topic_name")
        self.video_files = {self.topic_name[i]: video_file_list[i] for i in range(len(self.topic_name))}
        self.bridge = CvBridge()
