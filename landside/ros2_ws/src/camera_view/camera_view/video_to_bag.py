#!/usr/bin/env python3

import rclpy
from rclpy.serialization import serialize_message
from rclpy.duration import Duration
import rosbag2_py
import sys
import cv2
from camera_view.converter_interface import BagVideoConverter


class VideoToBag(BagVideoConverter):

    def __init__(self):
        super().__init__()
        self.writer = rosbag2_py.SequentialWriter()

    def run(self):
        """Converts an AVI file to a rosbag with synthetic time-stamps"""
        start_time = self.get_clock().now()
        self.writer.open(self.storage_options, self.converter_options)
        for topic, file in self.video_files.items():
            topic_metadata = rosbag2_py._storage.TopicMetadata(
                name=topic,
                type='sensor_msgs/msg/Image',
                serialization_format=self.SERIALIZATION_FORMAT)
            self.writer.create_topic(topic_metadata)
            cap = cv2.VideoCapture(file)
            fps = cap.get(cv2.CAP_PROP_FPS)
            counter = 0
            while cap.isOpened():
                ret, frame = cap.read()
                if not ret:  # Frame not read correctly
                    break
                img = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                self.get_logger().info(f'Bagging frame {counter}')
                img.header.stamp = (start_time + Duration(seconds=(counter / fps))).to_msg()
                self.writer.write(topic, serialize_message(img), img.header.stamp.sec)
                counter += 1
            cap.release()


def main(args=None):
    try:
        rclpy.init(args=args)
        converter = VideoToBag()
        converter.run()
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        sys.exit(1)
    finally:
        converter.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
