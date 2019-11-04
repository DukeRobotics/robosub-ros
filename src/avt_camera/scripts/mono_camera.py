#!/usr/bin/env python3.7

from pymba import Vimba, __version__
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
class MonoCamera:
    TOPIC_NAME = 'mono_camera/raw'
    NODE_NAME = 'mono_camera'
    

    def __init__(self):
        rospy.init_node(self.NODE_NAME)
        self._pub = rospy.Publisher(self.TOPIC_NAME, Image, queue_size=10)
        self._bridge = CvBridge()
        self.TIMEOUT = rospy.get_param('~timeout', 10)

    def run(self):
        rate = rospy.Rate(1/self.TIMEOUT)
        with Vimba() as vimba:
            camera = vimba.camera(0)
            camera.open()

            # arm the camera and provide a function to be called upon frame ready
            camera.arm('Continuous', publish_frame)
            camera.start_frame_acquisition()
            while not rospy.is_shutdown():
                rospy.spin()
            camera.stop_frame_acquisition()
            camera.disarm()
            camera.close()
    def publish_frame(frame):
        image = frame.buffer_data_numpy()
        self._pub.publish(self._bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        
if __name__ == '__main__':
    try:
    	MonoCamera().run()
    except rospy.ROSInterruptException:
    	pass