#!/usr/bin/env python

from pymba import *
import rospy
from sensor_msgs.msg import Image, CameraInfo
from camera import Camera

class MonoCamera:
    RAW_TOPIC_NAME = 'raw'
    INFO_TOPIC_NAME = 'camera_info'
    NODE_NAME = 'mono_camera'

    def __init__(self):
        rospy.init_node(self.NODE_NAME)
        pub = rospy.Publisher(rospy.get_name() + "/" + self.RAW_TOPIC_NAME, Image, queue_size=10)
        info_pub = rospy.Publisher(rospy.get_name() + "/" + self.INFO_TOPIC_NAME, CameraInfo, queue_size=10)
        camera_id = rospy.get_param('~camera_id', None)
        self._camera = Camera(pub, info_pub, rospy.get_name(), camera_id)

    def run(self):
        with Vimba() as vimba:
            self._camera.initialize_camera(vimba)
            self._camera.start_capture()
            self._camera.start_acquisition()

            while not rospy.is_shutdown():
                self._camera.queue_frame_capture()
                self._camera.get_frame_data()
                time = rospy.Time.now()
                self._camera.publish_image(time)

            self._camera.stop_acquisition()

if __name__ == '__main__':
    try:
        MonoCamera().run()
    except rospy.ROSInterruptException:
        pass
