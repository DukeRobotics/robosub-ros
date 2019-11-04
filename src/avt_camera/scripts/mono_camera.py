#!/usr/bin/env python

from pymba import *
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from time import sleep
import sys
class MonoCamera:
    TOPIC_NAME = 'mono_camera/raw'
    NODE_NAME = 'mono_camera'
    

    def __init__(self):
        rospy.init_node(self.NODE_NAME)
        self._pub = rospy.Publisher(self.TOPIC_NAME, Image, queue_size=10)
        self._bridge = CvBridge()
        self.TIMEOUT = rospy.get_param('~timeout', 10)

    def run(self):
        
        with Vimba() as vimba:
            system = vimba.getSystem()
            system.runFeatureCommand("GeVDiscoveryAllOnce")
            sleep(0.2)

            camera_ids = vimba.getCameraIds()
            if not camera_ids:
                rospy.logerr("No cameras found, big RIP")
                sys.exit(0)


            for cam_id in camera_ids:
                rospy.loginfo("Camera found: ", cam_id)

            c0 = vimba.getCamera(camera_ids[0])
            c0.openCamera()

            try:
                #gigE camera
                print("Packet size:", c0.GevSCPSPacketSize)
                c0.StreamBytesPerSecond = 100000000
                print("BPS:", c0.StreamBytesPerSecond)
            except:
                #not a gigE camera
                pass

            #set pixel format
            c0.PixelFormat = "BGR8Packed"  # OPENCV DEFAULT
            c0.AcquisitionMode = "Continuous"
            sleep(0.2)
            frame = c0.getFrame()
            frame.announceFrame()

            c0.startCapture()

            framecount = 0
            droppedframes = []
            
            c0.runFeatureCommand("AcquisitionStart")

            rate = rospy.Rate(1/self.TIMEOUT)
            while not rospy.is_shutdown():
                try:
                    frame.queueFrameCapture()
                    success = True
                except:
                    droppedframes.append(framecount)
                    success = False
                
                
                frame.waitFrameCapture(1000)
                frame_data = frame.getBufferByteData()
                
                if success:
                    img = np.ndarray(buffer=frame_data,
                                    dtype=np.uint8,
                                    shape=(frame.height, frame.width, frame.pixel_bytes))
                    self._pub.publish(self._bridge.cv2_to_imgmsg(img, "bgr8"))
                rospy.sleep(rate)
            c0.runFeatureCommand("AcquisitionStop")
            c0.endCapture()
            c0.revokeAllFrames()
            c0.closeCamera()

if __name__ == '__main__':
    try:
    	MonoCamera().run()
    except rospy.ROSInterruptException:
    	pass