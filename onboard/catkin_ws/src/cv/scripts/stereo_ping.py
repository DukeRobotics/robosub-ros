import rospy
import os

from custom_msgs.srv import StartPing
from diagnostic_msgs.ms import DiagnosticStatus


class StereoPing:
    def __init__(self):
        pass

    def start_ping(self):
        hostname = "google.com"
        response = os.system("ping -c 1 " + hostname)

        pub = rospy.Publisher("cv/front/detections", DiagnosticStatus, queue_size=10)
        pub.publish(DiagnosticStatus(name='ping', message=response))

    def run(self):
        rospy.Service("ping_stereo", StartPing, self.start_ping)
        rospy.spin()
