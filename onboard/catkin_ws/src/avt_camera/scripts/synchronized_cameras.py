#!/usr/bin/env python

from pymba import *  # noqa
import rospy
from sensor_msgs.msg import Image, CameraInfo
from camera import Camera


class SynchronizedCameras:

    NODE_NAME = 'stereo'

    def __init__(self):
        rospy.init_node(self.NODE_NAME, anonymous=True)
        cam_name = rospy.get_param('~camera', ['camera'])
        cam_ids = rospy.get_param('~camera_id', dict.fromkeys(cam_name))
        self._cameras = []
        for cam in cam_name:
            image_topic = '/camera/{}/image_raw'.format(cam)
            info_topic = '/camera/{}/camera_info'.format(cam)
            img_pub = rospy.Publisher(image_topic, Image, queue_size=10)
            info_pub = rospy.Publisher(info_topic, CameraInfo, queue_size=10)
            self._cameras.append(Camera(img_pub, info_pub, cam, cam_ids[cam]))

    def for_each_camera(self, fn):
        for camera in self._cameras:
            fn(camera)

    def run(self):
        with Vimba() as vimba:  # noqa
            self.for_each_camera(lambda camera: camera.initialize_camera(vimba))
            self.for_each_camera(lambda camera: camera.start_capture())
            self.for_each_camera(lambda camera: camera.start_acquisition())

            while not rospy.is_shutdown():
                self.for_each_camera(lambda camera: camera.queue_frame_capture())
                self.for_each_camera(lambda camera: camera.get_frame_data())
                time = rospy.Time.now()
                self.for_each_camera(lambda camera: camera.publish_image(time))

            self.for_each_camera(lambda camera: camera.stop_acquisition())


if __name__ == '__main__':
    try:
        SynchronizedCameras().run()
    except rospy.ROSInterruptException:
        pass
