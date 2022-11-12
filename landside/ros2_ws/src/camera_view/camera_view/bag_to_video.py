#!/usr/bin/env python3

import rclpy
from rclpy.serialization import deserialize_message
from rclpy.duration import Duration
from sensor_msgs.msg import Image
import rosbag2_py
import sys
import numpy as np
import cv2
from camera_view.converter_interface import BagVideoConverter


class BagToVideo(BagVideoConverter):

    def __init__(self):
        super().__init__()
        self.reader = rosbag2_py.SequentialReader()
        self.storage_filter = rosbag2_py.StorageFilter(topics=self.topics)

    def _get_info(self):
        """Calculates some information about the rosbag. Rosbags can be composed of multiple topics.
        Each topic contains a sequence of messages. In order to convert each topic to video files, 
        we need to have:
            1. The frame-rate of the video, or the the maximum publishing rate of the topic. This 
                is equivalent to the the minimum time interval between messages.
            2. The size of the image
            3. The number of times each image is repeated to match the frame-rate of the video. For
                instance, if the max frame-rate is 1 FPS and we have a gap of 10 seconds between two
                images, we know that the first image must be repeated for 10 frames to match the FPS.
        """
        times = {}
        sizes = {}
        while self.reader.has_next():
            (topic, data, t) = self.reader.read_next()
            msg = deserialize_message(data, Image)
            if topic not in times:
                times[topic] = []
            time = msg.header.stamp.sec + (msg.header.stamp.nanosec * 1e-9)
            times[topic].append(time)
            sizes[topic] = (msg.width, msg.height)
        intervals = {topic: np.diff(time) for (topic, time) in times.items()}
        nframes = {topic: np.int64(np.round(interval / min(interval)))
                   for (topic, interval) in intervals.items()}
        maxrates = {topic: np.max(1 / interval) for (topic, interval) in intervals.items()}
        return maxrates, sizes, nframes

    def _write_video(self, maxrates, sizes, nframes):
        """Writes a rosbag to a video file

        Args:
            maxrates: Mapping of topic to maximum frame-rate of rosbag
            sizes: Mapping of topic to image size
            nframes: Mapping of topic to number of frames each image should be repeated
        """
        for topic, video_file in self.video_files.items():
            writer = cv2.VideoWriter(video_file, cv2.VideoWriter_fourcc(*'DIVX'),
                                     np.ceil(maxrates[topic]), sizes[topic])
            counter = 0
            while self.reader.has_next():
                self.get_logger().info(f'Writing frame {counter}')
                (t, data, time) = self.reader.read_next()
                msg = deserialize_message(data, Image)
                img = np.asarray(self.bridge.imgmsg_to_cv2(msg, 'bgr8'))
                if counter == len(nframes[topic]):
                    writer.write(img)
                    break
                for _ in range(nframes[topic][counter]):
                    writer.write(img)
                counter += 1
            writer.release()

    def _init_reader(self):
        self.reader.open(self.storage_options, self.converter_options)
        self.reader.set_filter(self.storage_filter)

    def run(self):
        self._init_reader()
        maxrates, sizes, nframes = self._get_info()
        self._init_reader()
        self._write_video(maxrates, sizes, nframes)


def main(args=None):
    try:
        rclpy.init(args=args)
        converter = BagToVideo()
        converter.run()
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        raise
    finally:
        converter.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
