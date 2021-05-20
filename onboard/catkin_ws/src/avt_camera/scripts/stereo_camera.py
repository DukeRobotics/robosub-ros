#!/usr/bin/env python3

from vimba import *  # noqa
import rospy
from sensor_msgs.msg import Image, CameraInfo
from camera import Camera
import threading


class StereoCamera:

    NODE_NAME = 'stereo'

    def __init__(self):
        rospy.init_node(self.NODE_NAME, anonymous=True)
        cam_name = rospy.get_param('~camera', ['camera'])
        cam_ids = rospy.get_param('~camera_id', dict.fromkeys(cam_name))
        self._cameras = []
        for cam in cam_name:
            image_topic = f'/camera/{cam}/image_raw'
            info_topic = f'/camera/{cam}/camera_info'
            img_pub = rospy.Publisher(image_topic, Image, queue_size=10)
            info_pub = rospy.Publisher(info_topic, CameraInfo, queue_size=10)
            self._cameras.append(Camera(img_pub, info_pub, cam, cam_ids[cam]))

    def for_each_camera(self, fn):
        for camera in self._cameras:
            fn(camera)

    def run(self):
        with Vimba.get_instance():  # noqa
            self.for_each_camera(lambda camera: camera.initialize_camera())
            threads = []
            event = threading.Event()
            for camera in self._cameras:
                threads.append(threading.Thread(target=camera.capture, args=(event,)))
            
            for thread in threads:
                thread.start()
            rospy.spin()
            event.set()
            for thread in threads:
                thread.join()


if __name__ == '__main__':
    try:
        StereoCamera().run()
    except rospy.ROSInterruptException:
        pass
