#!/usr/bin/env python

import rospy
import psutil
import GPUtil
from custom_msgs.msg import Memory, SystemUsage

class SystemInfoPublisher:
    TOPIC_NAME = 'system/usage'
    NODE_NAME = 'dvl_raw_publisher'
    def __init__(self):
        self._pub = rospy.Publisher(self.TOPIC_NAME, SystemUsage, queue_size=10)
        self._current_msg = SystemUsage()
        self.gpu_memory = Memory()
        self.ram_memory = Memory()
        self.disk_memory = Memory()

    def get_gpu_memory_others(self):
        GPUs = GPUtil.getGPUs()
        if (GPUs > 0):
            gpu = GPUs[0]
            self.gpu_memory.used = gpu.memoryUsed
            self.gpu_memory.total = gpu.memoryTotal
            self.gpu_memory.percentage = gpu.memoryUsed/gpu.memoryTotal * 100
            self._current_msg.gpu_percent = gpu.load * 100
            self._current_msg.gpu_speed = 0
        else:
            self.gpu_memory.used = 0
            self.gpu_memory.total = 0
            self.gpu_memory.percentage = 0
        return self.gpu_memory


    def get_ram_memory(self):
        self.ram_memory.used = psutil.virtual_memory().total - psutil.virtual_memory().available
        self.ram_memory.total = psutil.virtual_memory().total
        self.ram_memory.percentage = psutil.virtual_memory().percent
        return self.ram_memory

    def get_disk_memory(self):
        self.disk_memory.used = psutil.disk_usage('/').used
        self.disk_memory.total = psutil.disk_usage('/').total * 0.95
        self.disk_memory.percentage = psutil.disk_usage('/').percent
        return self.disk_memory

    def run(self):
        rospy.init_node(self.NODE_NAME)
        while not rospy.is_shutdown():
            gpu = self.get_gpu_memory_others()
            ram = self.get_ram_memory()
            disk = self.get_disk_memory()

            self._current_msg.ram = ram

            self._current_msg.disk = disk
            self._current_msg.cpu_percent = psutil.cpu_percent(interval=0.5)
            self._current_msg.cpu_speed = psutil.cpu_freq().current

            self._current_msg.gpu_memory = gpu

            self._current_msg.header.stamp = rospy.Time.now()
            self._current_msg.header.frame_id = "system_usage_link"
            self._pub.publish(self._current_msg)

if __name__ == '__main__':
	try:
		SystemInfoPublisher().run()
	except rospy.ROSInterruptException:
		pass
