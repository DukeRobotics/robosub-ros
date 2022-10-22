#!/usr/bin/env python3

from brping import Ping360
import numpy as np

class Sonar:
    """Class to interface with the Sonar device.
    """

    SERIAL_PORT_NAME = "/dev/ttyUSB2"  # PORT of the salea is ttyUSB2 for testing
    BAUD_RATE = 2000000  # hz
    SAMPLE_PERIOD_TICK_DURATION = 25e-9  # s
    SPEED_OF_SOUND_IN_WATER = 1480  # m/s
    FILTER_INDEX = 100 #number of values to filter TODO figure out where the noise starts

    FTDI_STR = "DK0C1WF7"

    def __init__(self, range, number_of_samples=1200, serial_port_name=SERIAL_PORT_NAME, baud_rate=BAUD_RATE):
        self.ping360 = Ping360()
        self.ping360.connect_serial(serial_port_name, baud_rate)  # TODO: Add try except for connecting to device
        self.ping360.initialize()
        period_and_duration = self.range_to_period_and_duration(range)

        self.number_of_samples = number_of_samples
        self.ping360.set_number_of_samples(number_of_samples)
        self.sample_period = period_and_duration[0]
        self.ping360.set_sample_period(self.sample_period)
        self.transmit_duration = period_and_duration[1]
        self.ping360.set_transmit_duration(self.transmit_duration)

    def range_to_period_and_duration(self, range):
        """From a given range determines the sample_period and transmit_duration

        Based off of sample data, the period and duration have the following relationship with the range:
            44.4*range = sample_period
            5.3*range = transmit_duration
        These were calculated using the pingviewer application from bluerobotics

        Args:
            range (int): max range in meters of the sonar scan
        
        Returns:
            tuple (int, int): tuple of (sample_period, transmit_duration)
        
        """
        return (int(44.4*range), int(5.3*range))


    def set_new_range(self, range):
        """Sets a new sample_period and transmit_duration

        Args:
            range (int): max range in meters of the sonar scan
        
        Returns:
            Nothing
        """
        period_and_duration = self.range_to_period_and_duration(range)
        self.sample_period = period_and_duration[0]
        self.ping360.set_sample_period(self.sample_period)
        self.transmit_duration = period_and_duration[1]
        self.ping360.set_transmit_duration(self.transmit_duration)


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
        distance = self.SPEED_OF_SOUND_IN_WATER * ((self.sample_period * self.SAMPLE_PERIOD_TICK_DURATION) * sample_number) / 2.0
        return distance


    def get_range(self):
        """Get current range of the sonar device in meters.

        Returns:
            float: Range of device in meters.
        """
        last_sample_index = self.number_of_samples - 1
        return self.get_distance_of_sample(last_sample_index)


    def sweep_biggest_byte(self, start_angele, end_angle):
        """Get the index of the biggest value and angle value out of all angles in a sweep

        Returns:
            (index within angle, biggest value (byte), angle of biggest value)
        """
        biggest_byte_array = []
        for theta in range (start_angele, end_angle):
            biggest_byte = self.get_biggest_byte(theta)
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
        filteredbytes = split_bytes[self.FILTER_INDEX:]
        best = np.argmax(filteredbytes)
              #(index, value)
        return (best+self.FILTER_INDEX, filteredbytes[best])


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

    sonar = Sonar(range=5)
    sweep_data = sonar.sweep_biggest_byte(150, 250)  #90deg in front
    print(f"Distance to object: {sonar.get_distance_of_sample(sweep_data[0])} | Angle: {sweep_data[2]}")
