#!/usr/bin/env python3

import rospy
import subprocess

from diagnostic_msgs.msg import DiagnosticStatus, DiagnosticArray


class PingIP:
    def __init__(self):
        rospy.init_node('ping_ip')

        self.publisher = rospy.Publisher("ping_ip", DiagnosticArray, queue_size=10)
        self.hostname = rospy.get_param("~hostname")
        self.rate = rospy.get_param("~rate")

    def ping(self):
        loop_rate = rospy.Rate(self.rate)

        while not rospy.is_shutdown():
            proc = subprocess.Popen(["ping", "-c", "1", self.hostname], stdout=subprocess.PIPE, stderr=subprocess.PIPE)

            stdout, _ = proc.communicate()
            response = proc.returncode

            msg_status = DiagnosticStatus()
            msg_status.level = 0 if response == 0 else 2
            msg_status.name = self.hostname
            msg_status.message = stdout.decode('ASCII')

            msg_array = DiagnosticArray()
            msg_array.header.stamp = rospy.Time.now()
            msg_array.status = [msg_status]

            self.publisher.publish(msg_array)

            loop_rate.sleep()


if __name__ == '__main__':
    PingIP().ping()
