#!/usr/bin/env python

import rospy
from std_srvs.srv import SetBool
import subprocess

class SaleaeEnable:
    SERVICE_TOPIC = '/acoustics/toggle_saleae'

    def __init__(self):
        rospy.init_node('saleae_enable')
        rospy.Service(self.SERVICE_TOPIC, SetBool, self.toggle_saleae)

    def call_saleae_enable_script(self, enable=True):
        subprocess.run(['saleae.sh', 'enable' if enable else 'disable'])

    def toggle_saleae(self, req):
        if req.enable:
            rospy.loginfo('Enabling Saleae')
            self.call_saleae_enable_script(True)
        else:
            rospy.loginfo('Disabling Saleae')
            self.call_saleae_enable_script(False)
        return True

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = SaleaeEnable()
    node.run()
