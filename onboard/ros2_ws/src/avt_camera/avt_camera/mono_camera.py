#!/usr/bin/env python3

from vimba import *  # noqa
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from avt_camera.camera import Camera
import threading


class MonoCamera(Node):

    NODE_NAME = 'mono_camera'

    def __init__(self):
        super().__init__(self.NODE_NAME)
        camera_name = self.declare_parameter('camera', 'camera').value
        self._camera_id = self.declare_parameter('camera_id', 'camera_id').value

        image_topic = f'/camera/{camera_name}/image_raw'
        info_topic = f'/camera/{camera_name}/camera_info'
        img_pub = self.create_publisher(Image, image_topic, 10)
        info_pub = self.create_publisher(CameraInfo, info_topic, 10)

        self._camera = Camera(self, img_pub, info_pub, self.get_name(), self._camera_id)
        self._thread = None
        self._thread_lock = threading.Lock()

    def connection_handler(self, cam, event):

        if event == CameraEvent.Detected and (                                                          # noqa: F405
                cam.get_id() == self._camera.get_camera_id() or self._camera.get_camera_id() is None):
            with self._thread_lock:
                self._thread[1].set()
                self._thread[0].join()
                event = threading.Event()
                self._thread = (threading.Thread(target=self._camera.capture, args=(event,)), event)
                self._thread[0].start()

        elif event == CameraEvent.Missing and cam.get_id() == self._camera.get_camera_id():             # noqa: F405
            with self._thread_lock:
                self.get_logger().error(f"Lost camera {cam.get_id()}.")
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

    def teardown(self):
        vimba.register_camera_change_handler(self.connection_handler)
        vimba.unregister_camera_change_handler(self.connection_handler)

        with self._thread_lock:
            self._thread[1].set()
            self._thread[0].join()


def main(args=None):
    try:
        rclpy.init(args=args)
        camera = MonoCamera()
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
