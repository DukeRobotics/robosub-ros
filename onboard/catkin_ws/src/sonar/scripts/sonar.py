#!/usr/bin/env python3

from brping import Ping360
#import rospy
import numpy as np

SERIAL_PORT_NAME = "COM4"  # TODO determine what port this is for the robot.
BAUD_RATE = 115200  # hz
SAMPLE_PERIOD_TICK_DURATION = 25e-9  # s
SPEED_OF_SOUND_IN_WATER = 1480  # m/s

NODE_NAME = "sonar"


class Sonar:
    """Class to interface with the Sonar device.
    """

    def __init__(self, sample_period, number_of_samples=1200, serial_port_name=SERIAL_PORT_NAME, baud_rate=BAUD_RATE):
        self.ping360 = Ping360()
        self.ping360.connect_serial(serial_port_name, baud_rate)  # TODO: Add try except for connecting to device

        self.is_initialized()

        self.number_of_samples = number_of_samples
        self.ping360.set_number_of_samples(number_of_samples)
        self.sample_period = sample_period
        self.ping360.set_sample_period(sample_period)

        #rospy.init_node(NODE_NAME)

    def is_initialized(self):
        """Checks if the sonar device is initialized.

        Returns:
            bool: True if the sonar device is initialized
        """
        return self.ping360.initialize()

    def request_data_at_angle(self, angle_in_gradians):
        """Set sonar device to provided angle and retrieve data.

        Args:
            angle_in_gradians (float): Angle 

        Returns:
            dictionary: Response from the device. See https://docs.bluerobotics.com/ping-protocol/pingmessage-ping360/#get for information about the dictionary response.
        """
        response = self.ping360.transmitAngle(angle_in_gradians)
        return response

    def get_distance_of_sample(self, sample_index):
        """Get the distance in meters of a sample given its index in the data array returned from the device.

        Computes distance using formula from https://bluerobotics.com/learn/understanding-and-using-scanning-sonars/

        Args:
            sample_index (int): Index of the sample in the data array, from 0 to N-1, where N = number of samples.

        Returns:
            float: Distance in meters of the sample from the sonar device.
        """
        sample_number = sample_index + 1
        distance = SPEED_OF_SOUND_IN_WATER * ((self.sample_period * SAMPLE_PERIOD_TICK_DURATION) * sample_number) / 2.0
        return distance

    def get_range(self):
        """Get current range of the sonar device in meters.

        Returns:
            float: Range of device in meters.
        """
        last_sample_index = self.number_of_samples - 1
        return self.get_distance_of_sample(last_sample_index)

    def get_biggest_byte(self):
        """Get the biggest value of the byte array.

        Returns:
            int: index of the biggest value.
        """
        data = self.ping360.transmitAngle(0).data
        split_bytes = [data[i:i+1] for i in range(len(data))]
        #print(split_bytes)
        cur_byte = 0
        byte_index = 0
        for i in range(100, len(split_bytes) - 1):
            byte_int = int.from_bytes(split_bytes[i], "big")
            byte_distance = self.get_distance_of_sample(i)
            print(f"{byte_int} {byte_distance}")
            if byte_int > cur_byte:
                cur_byte = byte_int
                byte_index = i
        print(f"{cur_byte} {byte_index}")
        return(byte_index)


if __name__ == "__main__":
    sonar = Sonar(200, serial_port_name="COM3")
    index = sonar.get_biggest_byte()
    print(sonar.get_distance_of_sample(index))
