#!/usr/bin/env python3

from pymba import *  # noqa
import rospy
from sensor_msgs.msg import Image, CameraInfo
from camera import Camera


class MonoCamera:

    NODE_NAME = 'mono_camera'

    def __init__(self):
        rospy.init_node(self.NODE_NAME, anonymous=True)
        camera_name = rospy.get_param('~camera', 'camera')
        camera_id = rospy.get_param('~camera_id', None)

        image_topic = f'/camera/{camera_name}/image_raw'
        info_topic = f'/camera/{camera_name}/camera_info'

        img_pub = rospy.Publisher(image_topic, Image, queue_size=10)
        info_pub = rospy.Publisher(info_topic, CameraInfo, queue_size=10)
        self._camera = Camera(img_pub, info_pub, rospy.get_name(), camera_id)

    def run(self):
        with Vimba() as vimba:  # noqa
            self._camera.initialize_camera(vimba)
            self._camera.start_capture()
            rospy.spin()
            self._camera.stop_capture()


if __name__ == '__main__':
    try:
        MonoCamera().run()
    except rospy.ROSInterruptException:
        pass
