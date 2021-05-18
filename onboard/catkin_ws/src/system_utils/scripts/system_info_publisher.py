#!/usr/bin/env python3

import rospy
import psutil
import GPUtil
from custom_msgs.msg import SystemUsage


class SystemInfoPublisher:
    TOPIC_NAME = 'system/usage'
    NODE_NAME = 'system_usage_publisher'

    def __init__(self):
        self._pub = rospy.Publisher(self.TOPIC_NAME, SystemUsage, queue_size=10)
        self._current_msg = SystemUsage()
        rospy.init_node(self.NODE_NAME)

    def get_cpu(self):
        self._current_msg.cpu_percent = psutil.cpu_percent(interval=0.5)
        self._current_msg.cpu_speed = psutil.cpu_freq().current

    def get_gpu(self):
        GPUs = GPUtil.getGPUs()
        if len(GPUs) > 0:
            gpu = GPUs[0]
            self._current_msg.gpu_memory.used = gpu.memoryUsed / 1000
            self._current_msg.gpu_memory.total = gpu.memoryTotal / 1000
            self._current_msg.gpu_memory.percentage = gpu.memoryUsed / gpu.memoryTotal * 100
            self._current_msg.gpu_percent = gpu.load * 100
            self._current_msg.gpu_speed = 0
        else:
            self._current_msg.gpu_memory.used = 0
            self._current_msg.gpu_memory.total = 0
            self._current_msg.gpu_memory.percentage = 0

    def get_ram(self):
        self._current_msg.ram.used = (psutil.virtual_memory().total - psutil.virtual_memory().available) / (10**9)
        self._current_msg.ram.total = psutil.virtual_memory().total / (10**9)
        self._current_msg.ram.percentage = psutil.virtual_memory().percent

    def get_disk(self):
        self._current_msg.disk.used = psutil.disk_usage('/').used / (10**9)
        self._current_msg.disk.total = psutil.disk_usage('/').total * 0.95 / (10**9)
        self._current_msg.disk.percentage = psutil.disk_usage('/').percent

    def run(self):
        r = rospy.Rate(15)
        while not rospy.is_shutdown():
            self._current_msg = SystemUsage()
            self.get_cpu()
            self.get_gpu()
            self.get_ram()
            self.get_disk()

            self._pub.publish(self._current_msg)
            r.sleep()


if __name__ == '__main__':
    try:
        SystemInfoPublisher().run()
    except rospy.ROSInterruptException:
        pass
