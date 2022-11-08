#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import psutil
import GPUtil
from custom_msgs.msg import SystemUsage


class SystemInfo(Node):

    TOPIC_NAME = 'system/usage'
    NODE_NAME = 'system_usage_publisher'
    RUN_LOOP_RATE = 2

    def __init__(self):
        super().__init__(self.NODE_NAME)
        self._pub = self.create_publisher(SystemUsage, self.TOPIC_NAME, 10)
        self._current_msg = SystemUsage()
        self.timer = self.create_timer(1/self.RUN_LOOP_RATE, self.run)

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
            self._current_msg.gpu_speed = 0.0
        else:
            self._current_msg.gpu_memory.used = 0.0
            self._current_msg.gpu_memory.total = 0.0
            self._current_msg.gpu_memory.percentage = 0.0

    def get_ram(self):
        self._current_msg.ram.used = (psutil.virtual_memory().total -
                                      psutil.virtual_memory().available) / (10**9)
        self._current_msg.ram.total = psutil.virtual_memory().total / (10**9)
        self._current_msg.ram.percentage = psutil.virtual_memory().percent

    def get_disk(self):
        self._current_msg.disk.used = psutil.disk_usage('/').used / (10**9)
        self._current_msg.disk.total = psutil.disk_usage('/').total * 0.95 / (10**9)
        self._current_msg.disk.percentage = psutil.disk_usage('/').percent

    def run(self):
        self._current_msg = SystemUsage()
        self.get_cpu()
        self.get_gpu()
        self.get_ram()
        self.get_disk()
        self._pub.publish(self._current_msg)


def main(args=None):
    try:
        rclpy.init(args=args)
        system_info = SystemInfo()
        rclpy.spin(system_info)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        raise
    finally:
        system_info.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
