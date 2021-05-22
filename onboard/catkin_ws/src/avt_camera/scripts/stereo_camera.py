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
        self._cameras = {}
        self._threads = {}
        for cam in cam_name:
            image_topic = f'/camera/{cam}/image_raw'
            info_topic = f'/camera/{cam}/camera_info'
            img_pub = rospy.Publisher(image_topic, Image, queue_size=10)
            info_pub = rospy.Publisher(info_topic, CameraInfo, queue_size=10)
            self._cameras[cam_ids[cam]] = Camera(img_pub, info_pub, cam, cam_ids[cam])

        self._thread_lock = threading.Lock()

    def connection_handler(self, cam, event):

        if event == CameraEvent.Detected and cam.get_id() in self._threads:
            with self._thread_lock:
                self._threads[cam.get_id()][1].set()
                self._threads[cam.get_id()][0].join()
                event = threading.Event()
                self._threads[cam.get_id()] = (threading.Thread(
                    target=self._cameras[cam.get_id()].capture, args=(event,)), event)
                self._threads[cam.get_id()][0].start()

        elif event == CameraEvent.Missing and cam.get_id() in self._threads:
            with self._thread_lock:
                rospy.logerr(f"Lost camera {cam.get_id()}.")
                self._threads[cam.get_id()][1].set()
                self._threads[cam.get_id()][0].join()

    def run(self):
        with Vimba.get_instance() as vimba:  # noqa
            vimba.GeVDiscoveryAllDuration.set(1000)
            vimba.GeVDiscoveryAllAuto.run()
            with self._thread_lock:
                for id, camera in self._cameras.items():
                    event = threading.Event()
                    self._threads[id] = (threading.Thread(target=camera.capture, args=(event,)), event)
                    self._threads[id][0].start()

            vimba.register_camera_change_handler(self.connection_handler)
            rospy.spin()
            vimba.unregister_camera_change_handler(self.connection_handler)

            with self._thread_lock:
                for thread, event in self._threads.values():
                    event.set()
                    thread.join()


if __name__ == '__main__':
    try:
        StereoCamera().run()
    except rospy.ROSInterruptException:
        pass
