#!/usr/bin/env python

from pymba import *
import rospy
from sensor_msgs.msg import Image, CameraInfo
from camera import Camera

class SynchronousCamera:
    RAW_TOPIC_NAME = 'raw'
    INFO_TOPIC_NAME = 'camera_info'
    NODE_NAME = 'stereo'

    def __init__(self):
        rospy.init_node(self.NODE_NAME)
        cam_name = rospy.get_param("~cameras")
        cam_ids = rospy.get_param("~camera_id", dict.fromkeys(cam_name))
        self._cameras = []
        for cam in cam_name:
            pub = rospy.Publisher(rospy.get_name() + "/" + cam + "/" + self.RAW_TOPIC_NAME, Image, queue_size=10)
            info_pub = rospy.Publisher(rospy.get_name() + "/" + cam + "/" + self.INFO_TOPIC_NAME, CameraInfo, queue_size=10)
            self._cameras.append(Camera(pub, info_pub, cam, cam_ids[cam]))

    def for_each_camera(self, fn):
        for camera in self._cameras:
            fn(camera)

    def run(self):
        with Vimba() as vimba:
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
        SynchronousCamera().run()
    except rospy.ROSInterruptException:
        pass
