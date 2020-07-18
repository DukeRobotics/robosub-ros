#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge
import resource_retriever as rr


class BagVideoConverter:

    def __init__(self):
        rospy.init_node("bag_video_converter", anonymous=True)
        bag_res = 'package://camera_view/bag/{}'.format(rospy.get_param("~bag_file"))
        self.bag_file = rr.get_filename(bag_res, use_protocol=False)
        avi_res = 'package://camera_view/avi/{}'
        vid_file_list = [rr.get_filename(avi_res.format(f), use_protocol=False) for f in rospy.get_param("~video_file")]
        self.topic_name = rospy.get_param("~topic_name")
        self.video_files = {self.topic_name[i]: vid_file_list[i] for i in range(len(self.topic_name))}
        self.bridge = CvBridge()
