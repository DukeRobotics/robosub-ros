#!/usr/bin/env python

from pymba import *
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
import numpy as np
import sys
from camera_info_manager import CameraInfoManager
class StereoCamera:
	RAW_TOPIC_NAME = 'image_raw'
	INFO_TOPIC_NAME = 'camera_info'
	NODE_NAME = 'stereo'
    

	def __init__(self):
		rospy.init_node(self.NODE_NAME)
		print(rospy.get_name() + "/" + self.INFO_TOPIC_NAME)
		self._left_pub = rospy.Publisher(rospy.get_name()+"/left/" + self.RAW_TOPIC_NAME, Image, queue_size=10)
		self._left_info_pub = rospy.Publisher(rospy.get_name()+"/left" + "/" + self.INFO_TOPIC_NAME, CameraInfo, queue_size=10)
		self._right_pub = rospy.Publisher(rospy.get_name()+"/right/" + self.RAW_TOPIC_NAME, Image, queue_size=10)
		self._right_info_pub = rospy.Publisher(rospy.get_name()+"/right" + "/" + self.INFO_TOPIC_NAME, CameraInfo, queue_size=10)
		self._bridge = CvBridge()
        #self._camera_id = rospy.get_param('~camera_id', None)
		self._left_camera_id = "DEV_000F315C1ED5"
		self._right_camera_id = "DEV_000F315C1ED8"

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
		

			if self._left_camera_id is None :
				self._left_camera_id = camera_ids[0]
			elif self._left_camera_id not in camera_ids:
				rospy.logerr("Requested camera ID (" + self._left_camera_id + ") not found, sorry")
				sys.exit(0)
			self._left_info_manager = CameraInfoManager(cname=self._left_camera_id, namespace = "left", url="package://avt_camera/calibrations/${NAME}.yaml")
			self._left_info_manager.loadCameraInfo()
	    
			if self._right_camera_id is None :
				self._right_camera_id = camera_ids[0]
			elif self._right_camera_id not in camera_ids:
				rospy.logerr("Requested camera ID (" + self._right_camera_id + ") not found, sorry")
				sys.exit(0)
			self._right_info_manager = CameraInfoManager(cname=self._right_camera_id, namespace = "right", url="package://avt_camera/calibrations/${NAME}.yaml")
			self._right_info_manager.loadCameraInfo()
			c0 = vimba.getCamera(self._left_camera_id)
			c0.openCamera()
			c1 = vimba.getCamera(self._right_camera_id)
			c1.openCamera()

			try:
                #gigE camera
				rospy.loginfo("Packet Size: " + str(c0.GevSCPSPacketSize))
				rospy.loginfo("Stream Bytes Per Second: " + str(c0.StreamBytesPerSecond))
				c0.runFeatureCommand("GVSPAdjustPacketSize")
				c0.StreamBytesPerSecond = 100000000
				c1.runFeatureCommand("GVSPAdjustPacketSize")
				c1.StreamBytesPerSecond = 100000000
			except:
                #not a gigE camera
				pass

            #set pixel format
			c0.PixelFormat="Mono8"
			c0.AcquisitionMode = "Continuous"
			c0.ExposureAuto = "Continuous"
			c0.Width = 1210
			c0.Height = 760
			lframe = c0.getFrame()
			lframe.announceFrame()
			
			#set left (c1)
			c1.PixelFormat="Mono8"
			c1.AcquisitionMode="Continuous"
			c1.ExposureAuto="Continuous"
			c1.Width = 1210
			c1.Height = 760
			rframe = c1.getFrame()
			rframe.announceFrame()

			c0.startCapture()
			c1.startCapture()

			framecount = 0
			droppedframes = []
            
			c0.runFeatureCommand("AcquisitionStart") 
			c1.runFeatureCommand("AcquisitionStart")
			while not rospy.is_shutdown():
				lframe.queueFrameCapture()
				lframe.waitFrameCapture()
				rframe.queueFrameCapture()
				rframe.waitFrameCapture()
				
				lframe_data = lframe.getBufferByteData()
				rframe_data = rframe.getBufferByteData()

				time = rospy.Time.now()

				limg = np.ndarray(buffer=lframe_data,
                                dtype=np.uint8,
                                shape=(lframe.height, lframe.width, lframe.pixel_bytes))
				limg_message = self._bridge.cv2_to_imgmsg(limg, "mono8")
				rimg = np.ndarray(buffer=rframe_data,
								dtype=np.uint8,
								shape=(rframe.height, rframe.width, rframe.pixel_bytes))
				rimg_message = self._bridge.cv2_to_imgmsg(rimg, "mono8")
				
				rimg_message.header.stamp = time
				rimg_message.header.frame_id = "right"
				limg_message.header.stamp = time
				limg_message.header.frame_id = "left"
				c0_message = self._left_info_manager.getCameraInfo()
				c0_message.header.stamp = time
				c1_message = self._right_info_manager.getCameraInfo()
				c1_message.header.stamp = time
				self._left_pub.publish(limg_message)
				self._right_pub.publish(rimg_message)
				self._left_info_pub.publish(self._left_info_manager.getCameraInfo())
				self._right_info_pub.publish(self._right_info_manager.getCameraInfo())
			c0.runFeatureCommand("AcquisitionStop")
			c1.runFeatureCommand("AcquisitionStop")
			c0.endCapture()
			c1.endCapture()
			c0.revokeAllFrames()
			c1.revokeAllFrams()
			c0.closeCamera()
			c1.closeCamera()

if __name__ == '__main__':
    try:
    	StereoCamera().run()
    except rospy.ROSInterruptException:
    	pass
