#!/usr/bin/env python3

import rospy
from brping import Ping1D
import time
import serial.tools.list_ports as list_ports
from std_msgs.msg import Float64

class Ping1D:

    BAUD_RATE = 115200
    PING_INTERVAL = 100 # ms
    SPEED_OF_SOUND = 1482000 # mm/s
    RANGE_START = 0 # mm
    RANGE_END = 5000 # mm
    GAIN_SETTING = 6
    MODE_AUTO = 0
    CONFIDANCE_THRESHOLD = 50

    PING1D_FTDI_OOGWAY = "DK0C1WF7"
    _serial_port = None

    PING1D_DEST_TOPIC = 'sensors/ping/distance'

    NODE_NAME = "ping"

    def __init__(self):
        rospy.init_node(self.NODE_NAME)
        self.ping_publisher = rospy.Publisher(self.PING1D_DEST_TOPIC, Float64, queue_size=10)

        # Make a new Ping
        self.ping1D = Ping1D()
        try:
            self._serial_port = next(list_ports.grep(self.PING1D_FTDI_OOGWAY)).device
        except StopIteration:
            rospy.logerr("Ping1D not found. Go yell at Will.")
            rospy.signal_shutdown("Shutting down ping node.")

        self.ping1D.connect_serial(f'{self._serial_port}', 115200)

        while self.ping1D.initialize() is False:
            print("Failed to init Ping1D, retrying in 0.5 seconds... ")
            time.sleep(0.5)

        self.ping1D.set_speed_of_sound(self.SPEED_OF_SOUND) #mm/s
        self.ping1D.set_ping_interval(self.PING_INTERVAL)
        self.ping1D.set_mode_auto(self.MODE_AUTO)
        self.ping1D.set_gain_setting(self.GAIN_SETTING)
        self.ping1D.set_range(self.RANGE_START, self.RANGE_END)

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            data = self.ping1D.get_distance_simple()
            if data:
                distance = data["distance"]
                confidance = data["confidence"]
                if confidance > self.CONFIDANCE_THRESHOLD:
                    self.ping_publisher.publish(float(distance))
            else:
                rospy.logerr("Failed to get distance data from Ping1D")
            rate.sleep()

if __name__ == '__main__':
    try:
        Ping1D().run()
    except rospy.ROSInterruptException:
        pass