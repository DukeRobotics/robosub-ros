#!/usr/bin/env python3

from vimba import *  # noqa
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from avt_camera.camera import Camera
import threading


class StereoCamera(Node):

    NODE_NAME = 'stereo'

    def __init__(self):
        super().__init__(self.NODE_NAME)
        cam_names = self.declare_parameter('camera', ['camera']).value
        cam_ids = self.declare_parameter('camera_id', ['camera_id']).value
        self._cameras = {}
        self._threads = {}
        for cam, cam_id in zip(cam_names, cam_ids):
            image_topic = f'/camera/{cam}/image_raw'
            info_topic = f'/camera/{cam}/camera_info'
            img_pub = self.create_publisher(Image, image_topic, 10)
            info_pub = self.create_publisher(CameraInfo, info_topic, 10)
            self._cameras[cam_id] = Camera(self, img_pub, info_pub, cam, cam_id)

        self._thread_lock = threading.Lock()

    def connection_handler(self, cam, event):

        if event == CameraEvent.Detected and cam.get_id() in self._threads:                 # noqa: F405
            with self._thread_lock:
                self._threads[cam.get_id()][1].set()
                self._threads[cam.get_id()][0].join()
                event = threading.Event()
                self._threads[cam.get_id()] = (threading.Thread(
                    target=self._cameras[cam.get_id()].capture, args=(event,)), event)
                self._threads[cam.get_id()][0].start()

        elif event == CameraEvent.Missing and cam.get_id() in self._threads:                # noqa: F405
            with self._thread_lock:
                self.get_logger().error(f"Lost camera {cam.get_id()}.")
                self._threads[cam.get_id()][1].set()
                self._threads[cam.get_id()][0].join()

    def run(self):
        with Vimba.get_instance() as vimba:  # noqa
            vimba.GeVDiscoveryAllDuration.set(1000)
            vimba.GeVDiscoveryAllAuto.run()
            with self._thread_lock:
                for id, camera in self._cameras.items():
                    event = threading.Event()
                    self._threads[id] = (threading.Thread(
                        target=camera.capture, args=(event,)), event)
                    self._threads[id][0].start()

            vimba.register_camera_change_handler(self.connection_handler)

    def teardown(self):
        vimba.unregister_camera_change_handler(self.connection_handler)

        with self._thread_lock:
            for thread, event in self._threads.values():
                event.set()
                thread.join()


def main(args=None):
    try:
        rclpy.init(args=args)
        camera = StereoCamera()
        camera.run()
        rclpy.spin(camera)
        camera.teardown()
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        sys.exit(1)
    finally:
        camera.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
