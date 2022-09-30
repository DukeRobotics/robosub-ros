#!/usr/bin/env python3

from brping import Ping360  # TODO add this import dependency to dockerfile and update dependencies in CMakeLists.txt and package.xml
import rospy

SERIAL_PORT_NAME = "COM4"
BAUD_RATE = 115200  # hz
SAMPLE_PERIOD_TICK_DURATION = 25e-9  # s
SPEED_OF_SOUND_IN_WATER = 1480  # m/s

NODE_NAME = "sonar"


class Sonar:
    def __init__(self, serial_port_name=SERIAL_PORT_NAME, baud_rate=BAUD_RATE):
        self.ping360 = Ping360()
        self.ping360.connect_serial(serial_port_name, baud_rate)
        self.sample_period = 80  # Double check this
        
        rospy.init_node(NODE_NAME)

    def is_initialized(self):
        return self.ping360.initialize()

    def request_data_at_angle(self, angle_in_gradians):
        response = self.ping360.transmitAngle(angle_in_gradians)
        return response

    def get_distance_of_sample(self, sample_index):
        distance = (self.sample_period * SAMPLE_PERIOD_TICK_DURATION) * sample_index * SPEED_OF_SOUND_IN_WATER / 2.0
        return distance


if __name__ == "__main__":
    sonar = Sonar()
