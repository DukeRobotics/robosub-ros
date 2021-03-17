#!/usr/bin/env python

import rospy
import psutil
import GPUtil
from custom_msgs.msg import Memory, SystemUsage


class SystemInfoPublisher:
    TOPIC_NAME = 'system/usage'
    NODE_NAME = 'system_usage_publisher'

    def __init__(self):
        self._pub = rospy.Publisher(self.TOPIC_NAME, SystemUsage, queue_size=10)
        self._current_msg = SystemUsage()
        rospy.init_node(self.NODE_NAME)

    def get_gpu_memory_others(self):
        gpu_memory = Memory()
        GPUs = GPUtil.getGPUs()
        if (len(GPUs) > 0):
            gpu = GPUs[0]
            gpu_memory.used = gpu.memoryUsed
            gpu_memory.total = gpu.memoryTotal
            gpu_memory.percentage = gpu.memoryUsed/gpu.memoryTotal * 100
            self._current_msg.gpu_percent = gpu.load * 100
            self._current_msg.gpu_speed = 0
        else:
            gpu_memory.used = 0
            gpu_memory.total = 0
            gpu_memory.percentage = 0
        return gpu_memory

    def get_ram_memory(self):
        ram_memory = Memory()
        ram_memory.used = psutil.virtual_memory().total - psutil.virtual_memory().available
        ram_memory.total = psutil.virtual_memory().total
        ram_memory.percentage = psutil.virtual_memory().percent
        return ram_memory

    def get_disk_memory(self):
        disk_memory = Memory()
        disk_memory.used = psutil.disk_usage('/').used
        disk_memory.total = psutil.disk_usage('/').total * 0.95
        disk_memory.percentage = psutil.disk_usage('/').percent
        return disk_memory

    def run(self):
        while not rospy.is_shutdown():
            gpu = self.get_gpu_memory_others()
            ram = self.get_ram_memory()
            disk = self.get_disk_memory()

            self._current_msg.ram = ram

            self._current_msg.disk = disk
            self._current_msg.cpu_percent = psutil.cpu_percent(interval=0.5)
            self._current_msg.cpu_speed = psutil.cpu_freq().current

            self._current_msg.gpu_memory = gpu

            self._pub.publish(self._current_msg)


if __name__ == '__main__':
    try:
        SystemInfoPublisher().run()
    except rospy.ROSInterruptException:
        pass
