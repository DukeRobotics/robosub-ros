#!/usr/bin/env python3

from brping import Ping360
import numpy as np
import decodePingPythonPing360

SERIAL_PORT_NAME = "COM4"  # TODO determine what port this is for the robot.
BAUD_RATE = 115200  # hz
SAMPLE_PERIOD_TICK_DURATION = 25e-9  # s
SPEED_OF_SOUND_IN_WATER = 1480  # m/s
FILTER_INDEX = 100 #number of values to filter TODO figure out where the noise starts

class Sonar:
    """Class to interface with the Sonar device.
    """

    def __init__(self, sample_period, transmit_duration=11, number_of_samples=1200, serial_port_name=SERIAL_PORT_NAME, baud_rate=BAUD_RATE):
        self.ping360 = Ping360()
        self.ping360.connect_serial(serial_port_name, baud_rate)  # TODO: Add try except for connecting to device
        self.ping360.initialize()

        self.number_of_samples = number_of_samples
        self.ping360.set_number_of_samples(number_of_samples)
        self.sample_period = sample_period
        self.ping360.set_sample_period(sample_period)
        self.transmit_duration = transmit_duration
        self.ping360.set_transmit_duration(transmit_duration)

        #rospy.init_node(NODE_NAME)

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

    def sweep(self, start_angele, end_angle):
        """Get the index of the biggest value and angle value out of all angles in a sweep

        Returns:
            (index within angle, biggest value (byte), angle of biggest value)
        """
        biggest_byte_array = []
        for theta in range (start_angele, end_angle):
            data = sonar.request_data_at_angle(theta)
            biggest_byte = sonar.get_biggest_byte(data)
            biggest_byte_array.append(biggest_byte + (theta,))
        max_tup = max(biggest_byte_array, key=lambda tup: tup[1])
        #      (index, byte, angle)
        return max_tup

    def get_biggest_byte(self, angle):
        """Get the biggest value of the byte array.

        Returns:
            (biggest value index, biggest value)
        """
        data = self.ping360.transmitAngle(angle).data
        split_bytes = [data[i:i+1] for i in range(len(data))]
        filteredbytes = split_bytes[FILTER_INDEX:]
        best = np.argmax(filteredbytes)
              #(index,             value)
        return (best+FILTER_INDEX, filteredbytes[best])

    #for local testing
    def get_biggest_byte_data(self, data):
        """Get the biggest value of the byte array with preloaded data.

        Returns:
            (biggest value index, biggest value)
        """
        split_bytes = [data[i:i+1] for i in range(len(data))]
        filteredbytes = split_bytes[FILTER_INDEX:]
        best = np.argmax(filteredbytes)
              #(index,             value)
        return (best+FILTER_INDEX, int.from_bytes(filteredbytes[best],"little"))


if __name__ == "__main__":
    #settings for a 10m scan:
    #   transmit_duration = 53
    #   sample_period = 444
    #   transmit_frequency = 750
    #   BAUD_RATE = 2000000

    #settings for 5m scan:
    #   transmit_duration = 27
    #   sample_period = 222
    #   transmit_frequency = 750
    #   BAUD_RATE = 2000000

    sonar = Sonar(sample_period=222, transmit_duration=27, serial_port_name="COM3", baud_rate=2000000)

    #sampleData = decodePingPythonPing360.getdecodedfile('\\SampleTylerData.bin')
    #Below is for testing with a given local file
    #biggestbytearray = []
    # for index, (timestamp, decoded_message) in enumerate(sampleData):
    #     if(index >= 49 and index <= 149):
    #         biggestByte = sonar.get_biggest_byte_data(decoded_message.data)
    #         #print(f"{biggestByte} {decoded_message.angle}")
    #         biggestbytearray.append(biggestByte + (decoded_message.angle,))
    #     #index of the biggest byte
    
    #Below is testing with the main sonar
    sweep_data = sonar.sweep(150, 250)  #90deg in front
    print(f"Distance to object: {sonar.get_distance_of_sample(sweep_data[0])} | Angle: {sweep_data[2]}")