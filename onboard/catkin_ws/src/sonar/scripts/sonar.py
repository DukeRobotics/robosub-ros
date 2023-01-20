#!/usr/bin/env python3

from brping import Ping360
import numpy as np
import sonar_utils
from geometry_msgs.msg import Pose
from sonar_image_processing import scan_and_build_sonar_image, find_gate_posts, find_buoy
import os
from tf import TransformListener


class Sonar:
    """Class to interface with the Sonar device.
    """

    IF_LOCAL_TEST = True #only for local testing with the sonar script

    SERIAL_PORT_NAME = "/dev/ttyUSB2"  # PORT of the salea is ttyUSB2 for testing
    BAUD_RATE = 2000000  # hz
    SAMPLE_PERIOD_TICK_DURATION = 25e-9  # s
    SPEED_OF_SOUND_IN_WATER = 1480  # m/s
    FILTER_INDEX = 100  # number of values to filter TODO figure out where the noise starts

    def __init__(self, range, number_of_samples=1200, serial_port_name=SERIAL_PORT_NAME, baud_rate=BAUD_RATE):
        if(self.IF_LOCAL_TEST):
            import rospy
            rospy.init_node('sonar')
        self.ping360 = Ping360()
        self.ping360.connect_serial(serial_port_name, baud_rate)  # TODO: Add try except for connecting to device
        # self.ping360.connect_udp(self.ETHERNET_PORT_NAME)
        self.ping360.initialize()
        period_and_duration = self.range_to_period_and_duration(range)

        self.number_of_samples = number_of_samples
        self.ping360.set_number_of_samples(number_of_samples)
        self.sample_period = period_and_duration[0]
        self.ping360.set_sample_period(self.sample_period)
        self.transmit_duration = period_and_duration[1]
        self.ping360.set_transmit_duration(self.transmit_duration)

        self.listener = TransformListener()

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
            dictionary: Response from the device.
            See https://docs.bluerobotics.com/ping-protocol/pingmessage-ping360/#get for information about the
            dictionary response.
        """
        response = self.ping360.transmitAngle(angle_in_gradians)
        return response

    def get_sweep(self, range_start, range_end):
        """ Get data along a range of angles

        Args:
            range_start (int, optional): Angle to start sweep in gradians. Defaults to 100.
            range_end (int, optional): Angle to end sweep in gradians. Defaults to 300.

        Returns:
            List: List of data messages from the Sonar device
        """
        data_list = []
        for i in range(range_start, range_end):
            data = self.request_data_at_angle(i).data
            data_list.append(data)
        return data_list

    def get_distance_of_sample(self, sample_index):
        """Get the distance in meters of a sample given its index in the data array returned from the device.

        Computes distance using formula from https://bluerobotics.com/learn/understanding-and-using-scanning-sonars/

        Args:
            sample_index (int): Index of the sample in the data array, from 0 to N-1, where N = number of samples.

        Returns:
            float: Distance in meters of the sample from the sonar device.
        """
        sample_number = sample_index + 1
        distance = self.SPEED_OF_SOUND_IN_WATER * \
            ((self.sample_period * self.SAMPLE_PERIOD_TICK_DURATION) * sample_number) / 2.0
        return distance

    def get_range(self):
        """Get current range of the sonar device in meters.

        Returns:
            float: Range of device in meters.
        """
        last_sample_index = self.number_of_samples - 1
        return self.get_distance_of_sample(last_sample_index)

    def sweep_biggest_byte(self, start_angle, end_angle):
        """ Get the index of the biggest value and angle value out of all angles in a sweep

        Args:
            start_angle (int): Angle to start sweep in gradians
            end_angle (int): Angle to end sweep in gradians

        Returns:
            Tuple: index within angle, biggest value (byte), angle of biggest value
        """
        biggest_byte_array = []
        for theta in range(start_angle, end_angle):
            biggest_byte = self.get_biggest_byte(theta)
            biggest_byte_array.append(biggest_byte + (theta,))
        max_tup = max(biggest_byte_array, key=lambda tup: tup[1])
        return max_tup

    def get_biggest_byte(self, angle):
        """ Get the biggest value of the byte array of data scanned at input angle

        Args:
            angle (int): Angle in gradians for where to measure with the sonar.

        Returns:
            Tuple: biggest value index, biggest value
        """
        data = self.ping360.transmitAngle(angle).data
        split_bytes = [data[i:i+1] for i in range(len(data))]
        filteredbytes = split_bytes[self.FILTER_INDEX:]
        best = np.argmax(filteredbytes)
        return (best+self.FILTER_INDEX, filteredbytes[best])

    def gradians_to_radians(self, angle_gradians):
        """ Converts gradians to radians

        Args:
            angle_gradians (float): Angle in gradians

        Returns:
            float: Angle in radians
        """
        angle_radians = (angle_gradians-200)*np.pi/200
        return angle_radians

    def to_robot_position(self, angle, index):
        """ Converts a point in sonar space a robot global position

        Args:
            angle (float): Angle in gradians of the point relative to in front of the sonar device
            index (int): Index of the data in the sonar response

        Returns:
            Pose: Pose in robot reference frame
        """
        # Need to change the static transform for where the sonar is on the robot
        x_pos = self.get_distance_of_sample(index) * np.cos(self.gradians_to_radians(200-angle))
        y_pos = self.get_distance_of_sample(index) * np.sin(self.gradians_to_radians(200-angle))
        print(f"{x_pos} {y_pos}")
        pos_of_point = Pose()
        pos_of_point.position.x = x_pos
        pos_of_point.position.y = y_pos
        pos_of_point.position.z = 0  # z cord isnt 0 as gate is a line
        pos_of_point.orientation.x = 0
        pos_of_point.orientation.y = 0
        pos_of_point.orientation.z = 0
        pos_of_point.orientation.w = 1

        global_pose = sonar_utils.transform_pose(self.listener, pos_of_point)

        return global_pose


def test_scan_and_finding_gate_posts():
    """ Test to do a scan with the sonar device and find gate posts from the resulting image """
    sonar = Sonar(range=5)
    JPEG_SAVE_PATH = os.path.join(os.path.dirname(__file__), 'sampleData', 'Sonar_Image_robot.jpeg')
    NPY_SAVE_PATH = os.path.join(os.path.dirname(__file__), 'sampleData', 'Sonar_Image_robot.npy')

    sonar_img = scan_and_build_sonar_image(sonar,
                                           display_results=False,
                                           npy_save_path=NPY_SAVE_PATH,
                                           jpeg_save_path=JPEG_SAVE_PATH)

    posts = find_gate_posts(sonar_img, display_results=False)
    print(posts)

import numpy as np
def test_gate_from_npy_file(file):
    img = np.load(file)
    img = img[:,150:]
    print(img)
    posts = find_gate_posts(img, display_results=True)
    print(posts)

def test_buoy_from_npy_file(file):
    img = np.load(file)
    print(img)
    posts = find_buoy(img, display_results=True)
    print(posts)

if __name__ == "__main__":
    # Settings for a 10m scan:
    #   transmit_duration = 53
    #   sample_period = 444
    #   transmit_frequency = 750
    #   BAUD_RATE = 2000000

    # Settings for 5m scan:
    #   transmit_duration = 27
    #   sample_period = 222
    #   transmit_frequency = 750
    #   BAUD_RATE = 2000000

    # sweep_data = sonar.sweep_biggest_byte(100, 300)  #180deg in front
    # print(f"Distance to object: {sonar.get_distance_of_sample(sweep_data[0])} | Angle: {sweep_data[2]}")

    # FOR STARTING A WEB SERVER IN FOLDER::: RUN "python -m http.server 8000"
    # test_buoy_from_npy_file(os.path.join(os.path.dirname(__file__), 'sampleData', 'gate.npy'))

    sonar = Sonar(5)
    print(sonar.to_robot_position(240, 4))
