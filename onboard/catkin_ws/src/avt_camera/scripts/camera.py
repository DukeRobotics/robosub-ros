from vimba import *  # noqa
import rospy
from cv_bridge import CvBridge
import sys
from camera_info_manager import CameraInfoManager


class Camera:

    def __init__(self, img_pub, info_pub, namespace, camera_id):
        self._img_pub = img_pub
        self._info_pub = info_pub
        self._bridge = CvBridge()
        self._camera_id = camera_id
        self._namespace = namespace
        self._info_manager = None
        self._c0 = None

    def get_camera(self):
        with Vimba.get_instance() as vimba:

            cameras = vimba.get_all_cameras()
            if not cameras:
                rospy.logerr("Cameras were not found.")
                sys.exit(1)

            for cam in cameras:
                rospy.loginfo("Camera found: " + cam.get_id())

            if self._camera_id is None:
                self._camera_id = cameras[0].get_id()
            elif self._camera_id not in (cam.get_id() for cam in cameras):
                rospy.logerr(f"Requested camera ID {self._camera_id} not found")
                sys.exit(1)

            self._c0 = vimba.get_camera_by_id(self._camera_id)
        self._info_manager = CameraInfoManager(cname=self._camera_id, namespace=self._namespace,
                                               url=f"package://avt_camera/calibrations/{self._camera_id}.yaml")

    def set_camera_settings(self):
        with self._c0:
            self._c0.GVSPAdjustPacketSize.run()
            while not self._c0.GVSPAdjustPacketSize.is_done():
                pass
            rospy.loginfo(f"{self._c0.get_pixel_formats()}")
            self._c0.StreamBytesPerSecond.set(124000000)
            self._c0.set_pixel_format(PixelFormat.BayerRG8)
            self._c0.AcquisitionMode.set("Continuous")
            self._c0.ExposureAuto.set("Continuous")
            self._c0.Width.set(1210)
            self._c0.Height.set(760)

    def initialize_camera(self):
        self.get_camera()
        self.set_camera_settings()

    def capture(self, killswitch):
        with self._c0:
            self._c0.start_streaming(handler=self.publish_image)
            killswitch.wait()
            self._c0.stop_streaming()

    def publish_image(self, cam, frame):
        rospy.loginfo(f"Received frame from camera {cam.get_id()}. Publishing to ROS")
        img_message = self._bridge.cv2_to_imgmsg(frame.as_numpy_ndarray(), "bayer_rggb8")
        img_message.header.stamp = rospy.Time.now()
        self._img_pub.publish(img_message)
        self._info_pub.publish(self._info_manager.getCameraInfo())
        cam.queue_frame(frame)
