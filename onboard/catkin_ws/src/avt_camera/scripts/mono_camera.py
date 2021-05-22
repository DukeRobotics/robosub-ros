#!/usr/bin/env python3

from vimba import *  # noqa
import rospy
from sensor_msgs.msg import Image, CameraInfo
from camera import Camera
import threading


class MonoCamera:

    NODE_NAME = 'mono_camera'

    def __init__(self):
        rospy.init_node(self.NODE_NAME, anonymous=True)
        camera_name = rospy.get_param('~camera', 'camera')
        
        self._camera_id = rospy.get_param('~camera_id', None)

        image_topic = f'/camera/{camera_name}/image_raw'
        info_topic = f'/camera/{camera_name}/camera_info'

        img_pub = rospy.Publisher(image_topic, Image, queue_size=10)
        info_pub = rospy.Publisher(info_topic, CameraInfo, queue_size=10)
        self._camera = Camera(img_pub, info_pub, rospy.get_name(), self._camera_id)
        self._thread = None
        self._thread_lock = threading.Lock()

    def connection_handler(self, cam, event):

        if event == CameraEvent.Detected and (cam.get_id() == self._camera.get_camera_id() or self._camera.get_camera_id() is None):
            with self._thread_lock:
                self._thread[1].set()
                self._thread[0].join()
                event = threading.Event()
                self._thread = (threading.Thread(target=self._camera.capture, args=(event,)), event)
                self._thread[0].start()
        
        elif event == CameraEvent.Missing and cam.get_id() == self._camera.get_camera_id():
            with self._thread_lock:
                rospy.logerr(f"Lost camera {cam.get_id()}.")
                self._thread[1].set()
                self._thread[0].join()

    def run(self):
        with Vimba.get_instance() as vimba:  # noqa
            vimba.GeVDiscoveryAllDuration.set(1000)
            vimba.GeVDiscoveryAllAuto.run()
            with self._thread_lock:
                event = threading.Event()
                self._thread = (threading.Thread(target=self._camera.capture, args=(event,)), event)
                self._thread[0].start()
            
            vimba.register_camera_change_handler(self.connection_handler)
            rospy.spin()
            vimba.unregister_camera_change_handler(self.connection_handler)

            with self._thread_lock:
                self._thread[1].set()
                self._thread[0].join()


if __name__ == '__main__':
    try:
        MonoCamera().run()
    except rospy.ROSInterruptException:
        pass
