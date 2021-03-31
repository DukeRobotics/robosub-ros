#!/usr/bin/env python

import rosbag
import numpy as np
import cv2
from converter_interface import BagVideoConverter
from itertools import izip


class BagToVideo(BagVideoConverter):

    def get_info(self, bag):
        times = {}
        sizes = {}
        for topic, msg, t in bag.read_messages(topics=self.topic_name):
            if topic not in times:
                times[topic] = []
            times[topic].append(msg.header.stamp.to_sec())
            sizes[topic] = (msg.width, msg.height)
        intervals = {topic: np.diff(time) for (topic, time) in times.items()}
        nframes = {topic: np.int64(np.round(10 * interval / min(interval))) for (topic, interval) in intervals.items()}
        maxrates = {topic: np.max(1 / interval) for (topic, interval) in intervals.items()}
        return maxrates, sizes, nframes

    def write_video(self, bag, maxrates, sizes, nframes):
        for topic, video_file in self.video_files.items():
            writer = cv2.VideoWriter(video_file, cv2.VideoWriter_fourcc(*'DIVX'),
                                     np.ceil(maxrates[topic] * 10), sizes[topic])
            iterator = bag.read_messages(topics=topic)
            for (t, msg, time), reps in izip(iterator, nframes[topic]):
                img = np.asarray(self.bridge.imgmsg_to_cv2(msg, 'rgb8'))
                for _ in range(reps):
                    writer.write(img)
            writer.release()

    def run(self):
        bag = rosbag.Bag(self.bag_file, 'r')
        maxrates, sizes, nframes = self.get_info(bag)
        self.write_video(bag, maxrates, sizes, nframes)
        bag.close()


if __name__ == '__main__':
    BagToVideo().run()
