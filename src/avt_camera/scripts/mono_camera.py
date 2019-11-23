#!/usr/bin/env python

from pymba import *
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
import numpy as np
import sys
from camera_info_manager import CameraInfoManager
class MonoCamera:
    RAW_TOPIC_NAME = 'raw'
    INFO_TOPIC_NAME = 'camera_info'
    NODE_NAME = 'mono_camera'
    

    def __init__(self):
        rospy.init_node(self.NODE_NAME)
	print(rospy.get_name() + "/" + self.INFO_TOPIC_NAME)
        self._pub = rospy.Publisher(rospy.get_name() + "/" + self.RAW_TOPIC_NAME, Image, queue_size=10)
        self._info_pub = rospy.Publisher(rospy.get_name() + "/" + self.INFO_TOPIC_NAME, CameraInfo, queue_size=10)
        self._bridge = CvBridge()
        self._camera_id = rospy.get_param('~camera_id', None)

    def run(self):
        
        with Vimba() as vimba:
            system = vimba.getSystem()
            system.runFeatureCommand("GeVDiscoveryAllOnce")
            rospy.sleep(0.1)

            camera_ids = vimba.getCameraIds()
            if not camera_ids:
                rospy.logerr("Sorry: Cameras were not found.")
                sys.exit(0)

            for cam_id in camera_ids:
                rospy.loginfo("Camera found: " + cam_id)

            if self._camera_id is None :
                self._camera_id = camera_ids[0]
            elif self._camera_id not in camera_ids:
                rospy.logerr("Requested camera ID (" + self._camera_id + ") not found, sorry")
                sys.exit(0)
            self._info_manager = CameraInfoManager(cname=self._camera_id, namespace = rospy.get_name(), url="package://avt_camera/calibrations/${NAME}.yaml")
            self._info_manager.loadCameraInfo()
            c0 = vimba.getCamera(self._camera_id)
            c0.openCamera()

            try:
                #gigE camera
                rospy.loginfo("Packet Size: " + str(c0.GevSCPSPacketSize))
                rospy.loginfo("Stream Bytes Per Second: " + str(c0.StreamBytesPerSecond))
                c0.runFeatureCommand("GVSPAdjustPacketSize")
                c0.StreamBytesPerSecond = 100000000
            except:
                #not a gigE camera
                pass

            #set pixel format
            c0.PixelFormat="Mono8"
            c0.AcquisitionMode = "Continuous"
            c0.ExposureAuto = "Continuous"
            c0.Width = 1210
            c0.Height = 760
            frame = c0.getFrame()
            frame.announceFrame()

            c0.startCapture()

            framecount = 0
            droppedframes = []
            
            c0.runFeatureCommand("AcquisitionStart")  
            while not rospy.is_shutdown():
                frame.queueFrameCapture()
                frame.waitFrameCapture()
                frame_data = frame.getBufferByteData()
                img = np.ndarray(buffer=frame_data,
                                dtype=np.uint8,
                                shape=(frame.height, frame.width, frame.pixel_bytes))
                img_message = self._bridge.cv2_to_imgmsg(img, "mono8")
		#img_message.header.stamp = rospy.Time.now()
                ci_message = self._info_manager.getCameraInfo()
		ci_message.header.stamp = rospy.Time.now()
		self._pub.publish(img_message)
                self._info_pub.publish(self._info_manager.getCameraInfo())
            c0.runFeatureCommand("AcquisitionStop")
            c0.endCapture()
            c0.revokeAllFrames()
            c0.closeCamera()

if __name__ == '__main__':
    try:
    	MonoCamera().run()
    except rospy.ROSInterruptException:
    	pass
