#!/usr/bin/env python

from pymba import *
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from time import sleep
import numpy as np
import sys
class MonoCamera:
    TOPIC_NAME = 'mono_camera/raw'
    NODE_NAME = 'mono_camera'
    

    def __init__(self):
        rospy.init_node(self.NODE_NAME)
        self._pub = rospy.Publisher(self.TOPIC_NAME, Image, queue_size=10)
        self._bridge = CvBridge()

    def run(self):
        
        with Vimba() as vimba:
            system = vimba.getSystem()
            system.runFeatureCommand("GeVDiscoveryAllOnce")
            rospy.sleep(0.1)

            camera_ids = vimba.getCameraIds()
            if not camera_ids:
                rospy.logerr("No cameras found, big RIP")
                sys.exit(0)

            for cam_id in camera_ids:
                rospy.loginfo("Camera found: " + str(cam_id))

            c0 = vimba.getCamera(camera_ids[0])
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
                self._pub.publish(self._bridge.cv2_to_imgmsg(img, "mono8"))
            c0.runFeatureCommand("AcquisitionStop")
            c0.endCapture()
            c0.revokeAllFrames()
            c0.closeCamera()

if __name__ == '__main__':
    try:
    	MonoCamera().run()
    except rospy.ROSInterruptException:
    	pass
