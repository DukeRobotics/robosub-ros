#!/usr/bin/env python

import rospy
import rosbag
import numpy as np
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from converter_interface import BagVideoConverter


class VideoToBag(BagVideoConverter):

    def create_bag(self, bag):
        start_time = rospy.Time.now()
        for topic, file in self.video_files.items():
            cap = cv2.VideoCapture(file)
            fps = cap.get(cv2.CAP_PROP_FPS)
            counter = 0
            while cap.isOpened():
                ret, frame = cap.read()
                if not ret:
                    # Frame not read correctly
                    break
                img = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                img.header.stamp = start_time + rospy.Duration(counter / fps)
                bag.write(topic, img, img.header.stamp)
                counter += 1
            cap.release()

    def run(self):
        bag = rosbag.Bag(self.bag_file, 'w')
        self.create_bag(bag)
        bag.close()


if __name__ == '__main__':
    VideoToBag().run()
