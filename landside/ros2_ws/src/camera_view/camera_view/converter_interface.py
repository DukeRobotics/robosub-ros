import rclpy
from rclpy.node import Node
import rosbag2_py
from cv_bridge import CvBridge
import resource_retriever as rr


class BagVideoConverter(Node):

    NODE_NAME = "bag_video_converter"
    SERIALIZATION_FORMAT = 'cdr'

    def __init__(self):
        super().__init__(self.NODE_NAME)

        bag_param = self.declare_parameter('bag_file', '').value
        bag_res = f'package://camera_view/bag/{bag_param}'
        self.bag_file = rr.get_filename(bag_res, use_protocol=False)

        avi_res = 'package://camera_view/avi/{}'
        video_param = self.declare_parameter('video_file', ['']).value
        vid_file_list = [rr.get_filename(avi_res.format(f), use_protocol=False)
                         for f in video_param]

        self.topics = self.declare_parameter('topic_name', ['']).value
        self.video_files = {self.topics[i]: vid_file_list[i] for i in range(len(self.topics))}
        self.bridge = CvBridge()
        self.storage_options = rosbag2_py._storage.StorageOptions(
            uri=self.bag_file,
            storage_id='sqlite3')
        self.converter_options = rosbag2_py._storage.ConverterOptions(
            self.SERIALIZATION_FORMAT, self.SERIALIZATION_FORMAT)
