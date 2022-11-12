from vimba import *  # noqa
from cv_bridge import CvBridge
from avt_camera.camera_info_manager import CameraInfoManager


class bcolors:
    OKGREEN = '\033[92m'
    RESET = '\033[0m'


class Camera:

    def __init__(self, node, img_pub, info_pub, namespace, camera_id):
        self.node = node
        self._img_pub = img_pub
        self._info_pub = info_pub
        self._bridge = CvBridge()
        self._camera_id = camera_id
        self._namespace = namespace
        if camera_id is not None:
            self._info_manager = CameraInfoManager(self.node, cname=self._camera_id, namespace=self._namespace,
                                                   url=f"package://avt_camera/calibrations/{self._camera_id}.yaml")
        self._c0 = None

    def get_camera_id(self):
        return self._camera_id

    def get_camera(self):
        with Vimba.get_instance() as vimba:     # noqa
            cameras = vimba.get_all_cameras()
            if not cameras:
                self.node.get_logger().error("Cameras were not found.")
                return False

            for cam in cameras:
                self.node.get_logger().info("Camera found: " + cam.get_id())

            if self._camera_id is None:
                self._camera_id = cameras[0].get_id()
                self._info_manager = CameraInfoManager(cname=self._camera_id, namespace=self._namespace,
                                                       url=f"package://avt_camera/calibrations/{self._camera_id}.yaml")
            elif self._camera_id not in (cam.get_id() for cam in cameras):
                self.node.get_logger().error(f"Requested camera ID {self._camera_id} not found.")
                return False
            self._c0 = vimba.get_camera_by_id(self._camera_id)
            self.node.get_logger().info(bcolors.OKGREEN + f"Connected camera {self._camera_id}." + bcolors.RESET)

        return True

    def set_camera_settings(self):
        with self._c0:
            self._c0.GVSPAdjustPacketSize.run()
            while not self._c0.GVSPAdjustPacketSize.is_done():
                pass
            self.node.get_logger().info(f"{self._c0.get_pixel_formats()}")
            self._c0.StreamBytesPerSecond.set(124000000)
            self._c0.set_pixel_format(PixelFormat.BayerRG8)     # noqa
            self._c0.AcquisitionMode.set("Continuous")
            self._c0.ExposureAuto.set("Continuous")
            self._c0.Width.set(1210)
            self._c0.Height.set(760)

    def capture(self, killswitch):
        if not self.get_camera():
            return
        self.set_camera_settings()
        with self._c0:
            self._c0.start_streaming(handler=self.publish_image)
            killswitch.wait()
            try:
                self._c0.stop_streaming()
            except Exception:
                pass

    def publish_image(self, cam, frame):
        self.node.get_logger().info(f"Received frame from camera {cam.get_id()}. Publishing to ROS")
        img_message = self._bridge.cv2_to_imgmsg(frame.as_numpy_ndarray(), "bayer_rggb8")
        img_message.header.stamp = rospy.Time.now()
        self._img_pub.publish(img_message)
        self._info_pub.publish(self._info_manager.getCameraInfo())
        cam.queue_frame(frame)
