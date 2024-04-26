#!/usr/bin/env python3

import rospy
from brping import Ping1D
import time
import serial.tools.list_ports as list_ports
from std_msgs.msg import Float64
import yaml
import os
import resource_retriever as rr


class Ping1DPublisher:

    BAUD_RATE = 115200
    PING_INTERVAL = 100  # ms
    SPEED_OF_SOUND = 1482000  # mm/s
    RANGE_START = 0  # mm
    RANGE_END = 5000  # mm
    GAIN_SETTING = 6
    MODE_AUTO = 0
    CONFIDANCE_THRESHOLD = 0

    CONFIG_FILE_PATH = f'package://data_pub/config/{os.getenv("ROBOT_NAME", "oogway")}.yaml'
    _serial_port = None

    PING1D_DEST_TOPIC = 'sensors/ping/distance'

    NODE_NAME = "ping"

    def __init__(self):
        rospy.init_node(self.NODE_NAME)
        self.ping_publisher = rospy.Publisher(self.PING1D_DEST_TOPIC, Float64, queue_size=10)

        with open(rr.get_filename(self.CONFIG_FILE_PATH, use_protocol=False)) as f:
            self._config_data = yaml.safe_load(f)

    def connect(self):
        # Make a new Ping
        self._ping1D = Ping1D()
        while self._serial_port is None and not rospy.is_shutdown():
            try:
                ping1D_ftdi_string = self._config_data['ping1D']['ftdi']
                self._serial_port = next(list_ports.grep(ping1D_ftdi_string)).device
            except StopIteration:
                rospy.logerr("Ping1D not found, trying again in 0.1 seconds.")
                rospy.sleep(0.1)

        self._ping1D.connect_serial(self._serial_port, 115200)

        while self._ping1D.initialize() is False:
            print("Failed to init Ping1D, retrying in 0.5 seconds... ")
            time.sleep(0.5)

        self._ping1D.set_speed_of_sound(self.SPEED_OF_SOUND)  # mm/s
        self._ping1D.set_ping_interval(self.PING_INTERVAL)
        self._ping1D.set_mode_auto(self.MODE_AUTO)
        self._ping1D.set_gain_setting(self.GAIN_SETTING)
        self._ping1D.set_range(self.RANGE_START, self.RANGE_END)

    def run(self):
        rate = rospy.Rate(10)
        self.connect()
        while not rospy.is_shutdown():
            data = self._ping1D.get_distance_simple()
            if data:
                distance = data["distance"]
                confidance = data["confidence"]
                if confidance >= self.CONFIDANCE_THRESHOLD:
                    self.ping_publisher.publish(float(distance/1000.0))
            else:
                rospy.logerr("Failed to get distance data from Ping1D")
            rate.sleep()


if __name__ == '__main__':
    try:
        Ping1DPublisher().run()
    except rospy.ROSInterruptException:
        pass
