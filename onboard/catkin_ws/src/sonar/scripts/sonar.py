#!/usr/bin/env python3

import rospy
from brping import Ping360
import numpy as np
import sonar_utils
import serial.tools.list_ports as list_ports
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from custom_msgs.msg import SonarSweepRequest
from sensor_msgs.msg import CompressedImage
from tf import TransformListener
from cv_bridge import CvBridge
from sonar_image_processing import build_color_sonar_image_from_int_array


class Sonar:
    """
    Class to interface with the Sonar device.
    """

    BAUD_RATE = 2000000  # hz
    SAMPLE_PERIOD_TICK_DURATION = 25e-9  # s
    SPEED_OF_SOUND_IN_WATER = 1482  # m/s

    # number of values to filter TODO figure out where the noise ends
    FILTER_INDEX = 100  # first 100 values are filtered out
    DEFAULT_RANGE = 5  # m
    DEFAULT_NUMER_OF_SAMPLES = 1200  # 1200 is max resolution

    SONAR_FTDI_OOGWAY = "DK0C1WF7"
    _serial_port = None

    SONAR_STATUS_TOPIC = 'sonar/status'
    SONAR_REQUEST_TOPIC = 'sonar/request'
    SONAR_RESPONSE_TOPIC = 'sonar/cv/response'
    SONAR_IMAGE_TOPIC = 'sonar/image/compressed'

    NODE_NAME = "sonar"

    CONSTANT_SWEEP_START = 100
    CONSTANT_SWEEP_END = 300

    def __init__(self):
        rospy.init_node(self.NODE_NAME)
        rospy.loginfo("starting sonar...")
        self.stream = rospy.get_param('~stream')
        self.debug = rospy.get_param('~debug')

        self.status_publisher = rospy.Publisher(self.SONAR_STATUS_TOPIC, String, queue_size=1)
        self.pub_response = rospy.Publisher(self.SONAR_RESPONSE_TOPIC, Pose, queue_size=1)

        # Enamble streaming
        self.cv_bridge = CvBridge()
        if self.stream:
            self.sonar_image_publisher = rospy.Publisher(self.SONAR_IMAGE_TOPIC, CompressedImage, queue_size=1)

        # Create ping360
        self.ping360 = Ping360()

        # Connect to the sonar
        try:
            self._serial_port = next(list_ports.grep(self.SONAR_FTDI_OOGWAY)).device
        except StopIteration:
            rospy.logerr("Sonar not found. Go yell at Will.")
            rospy.signal_shutdown("Shutting down sonar node.")

        self.ping360.connect_serial(f'{self._serial_port}', self.BAUD_RATE)
        self.ping360.initialize()

        # Setup default parameters
        self.number_of_samples = self.DEFAULT_NUMER_OF_SAMPLES
        self.ping360.set_number_of_samples(self.number_of_samples)

        self.sample_period = self.range_to_period(self.DEFAULT_RANGE)
        self.ping360.set_sample_period(self.sample_period)

        self.transmit_duration = self.range_to_transmit(self.DEFAULT_RANGE)
        self.ping360.set_transmit_duration(self.transmit_duration)

        self.listener = TransformListener()

    def range_to_period(self, range):
        """From a given range determines the sample_period

        sample_period is the time between each sample. given a
        distance we can calculate the sample period using the
        formula: 2 * range / (number_of_samples * speed_of_sound_in_water
        * 25e-9) where number of samples is the number of samples taken
        betweeen 0m and the set range, speed of sound in water is 1480m/s
        and 25e-9 is the sample period tick duration.
        https://discuss.bluerobotics.com/t/please-provide-some-answer-regards-ping360/6393/3

        Args:
            range (int): max range in meters of the sonar scan

        Returns:
            sample_period (int): sample period in ms

        """
        return round(2 * range / (self.number_of_samples * self.SPEED_OF_SOUND_IN_WATER *
                                  self.SAMPLE_PERIOD_TICK_DURATION))

    def range_to_transmit(self, range):
        """From a given range determines the transmit_duration

        Per firmware engineer:
        1. Starting point is TxPulse in usec = ((one-way range in metres) * 8000) /
        (Velocity of sound in metres per second)
        2. Then check that TxPulse is wide enough for currently selected sample interval in usec,
        i.e., if TxPulse < (2.5 * sample interval) then TxPulse = (2.5 * sample interval)
        3. Perform limit checking
        https://discuss.bluerobotics.com/t/please-provide-some-answer-regards-ping360/6393/3

        Args:
            range (int): max range in meters of the sonar scan

        Returns:
            transmit_duration (int): max transmit duration in ms

        """
        # 1
        transmit_duration = 8000 * range / self.SPEED_OF_SOUND_IN_WATER
        # 2 (transmit duration is microseconds, samplePeriod() is nanoseconds)
        transmit_duration = max(self.range_to_period(range) / 40, transmit_duration)
        # 3 min_transmit is 5 and max_transmit is 500
        return round(max(5, min(500, transmit_duration)))

    def set_new_range(self, range):
        """Sets a new sample_period and transmit_duration

        Args:
            range (int): max range in meters of the sonar scan

        Returns:
            Nothing
        """
        self.sample_period = self.range_to_period(range)
        self.ping360.set_sample_period(self.sample_period)

        self.transmit_duration = self.range_to_transmit(range)
        self.ping360.set_transmit_duration(self.transmit_duration)

    def meters_per_sample(self):
        """ Returns the target distance per sample, in meters.
        https://discuss.bluerobotics.com/t/access-ping360-data-for-post-processing-python/10416/2

        Args:
            None

        Returns:
            float: distance per sample

        """
        # sample_period is in 25ns increments
        # time of flight includes there and back, so divide by 2
        return self.SPEED_OF_SOUND_IN_WATER * self.sample_period * self.SAMPLE_PERIOD_TICK_DURATION / 2.0

    def get_distance_of_sample(self, sample_index):
        """Get the distance in meters of a sample given its index in the data
                array returned from the device.

        Computes distance using formula from
                https://bluerobotics.com/learn/understanding-and-using-scanning-sonars/

        Args:
            sample_index (int | float): Index of the sample in the data array,
            from 0 to N-1, where N = number of samples.

        Returns:
            float: Distance in meters of the sample from the sonar device.
        """
        # 0.5 for the average distance of sample
        return (sample_index + 0.5) * self.meters_per_sample()

    def request_data_at_angle(self, angle_in_gradians):
        """Set sonar device to provided angle and retrieve data.

        Args:
            angle_in_gradians (float): Angle

        Returns:
            dictionary: Response from the device. See
            https://docs.bluerobotics.com/ping-protocol/pingmessage-ping360/#get
            for information about the dictionary response.

            Basically, an array of number_of_samples size with each index having
            a distance of MAX_RANGE/number_of_samples I index meters from the
            sonar device. The value is the intensity of the ping at that point
        """
        response = self.ping360.transmitAngle(angle_in_gradians)
        response_to_int_array = [int(item) for item in response.data]  # converts bytestring to int array
        # replaces first FILTER_INDEX values with 0
        filtered_sonar_scan = [int(0)] * self.FILTER_INDEX + response_to_int_array[self.FILTER_INDEX:]
        return filtered_sonar_scan

    def get_sweep(self, range_start=100, range_end=300):
        """ Get data along a range of angles

        Args:
            range_start (int, optional): Angle to start sweep in gradians.
                    Defaults to 100. Min is 0
            range_end (int, optional): Angle to end sweep in gradians.
                    Defaults to 300. Max is 399.

        Returns:
            List: List of data messages from the Sonar device
            hi :) - VC
        """
        sonar_sweep_data = []
        for i in range(range_start, range_end + 1):  # inclusive ends
            sonar_scan = self.request_data_at_angle(i)
            sonar_sweep_data.append(sonar_scan)
        return np.vstack(sonar_sweep_data)

    def to_robot_position(self, angle, index):
        """ Converts a point in sonar space a robot global position

        Args:
            angle (float): Angle in gradians of the point relative to in front
                    of the sonar device
            index (int | float): Index of the data in the sonar response

        Returns:
            Pose: Pose in robot reference frame containing x and y position
                    of angle/index item
        """

        x_pos = self.get_distance_of_sample(index)*np.cos(
            sonar_utils.centered_gradians_to_radians(angle))
        y_pos = -1 * self.get_distance_of_sample(index)*np.sin(
            sonar_utils.centered_gradians_to_radians(angle))
        pos_of_point = Pose()
        pos_of_point.position.x = x_pos
        pos_of_point.position.y = y_pos
        pos_of_point.position.z = 0  # z cord is not really 0 but we don't care
        pos_of_point.orientation.x = 0
        pos_of_point.orientation.y = 0
        pos_of_point.orientation.z = 0
        pos_of_point.orientation.w = 1

        transformed_pose = sonar_utils.transform_pose(
            self.listener, pos_of_point)

        return transformed_pose

    def get_xy_of_object_in_sweep(self, start_angle, end_angle):
        """ Gets the depth of the sweep of a detected object. For now uses
            mean value

        Args:
            start_angle (int): Angle to start sweep in gradians
            end_angle (int): Angle to end sweep in gradians

        Returns:
            (Pose, List): Pose of the object in robot reference frame and
                          sonar sweep array
        """
        sonar_sweep_array = self.get_sweep(start_angle, end_angle)

        column_sums = np.sum(sonar_sweep_array, axis=0)
        average_column = np.sum(np.multiply(column_sums, np.arange(0, column_sums.size)))/np.sum(column_sums)

        row_sums = np.sum(sonar_sweep_array, axis=1)
        average_row = np.sum(np.multiply(row_sums, np.arange(0, row_sums.size)))/np.sum(row_sums)

        return (self.to_robot_position(average_row + start_angle, average_column), sonar_sweep_array)

    def convert_to_ros_compressed_img(self, sonar_sweep, compressed_format='jpg'):
        """ Convert any kind of image to ROS Compressed Image.

        Args:
            image (int): numpy array of int values representing the sonar image
            compressed_format (string): format to compress the image to

        Returns:
            CompressedImage: ROS Compressed Image message
        """
        sonar_image = build_color_sonar_image_from_int_array(sonar_sweep)
        return self.cv_bridge.cv2_to_compressed_imgmsg(sonar_image, dst_format=compressed_format)

    def constant_sweep(self, start_angle, end_angle, distance_of_scan):
        """ In debug mode scan indefinitely and publish images

        Args:
            start_angle (int): Angle to start sweep in gradians
            end_angle (int): Angle to end sweep in gradians
            distance_of_scan (int): Distance in meters to scan

        Returns:
            Nothing
        """
        rospy.loginfo("starting constant sweep")
        self.set_new_range(distance_of_scan)
        rospy.loginfo(f"stream: {self.stream}")

        # Scan indefinitely
        while not rospy.is_shutdown():
            try:
                rospy.loginfo(f"starting sweep from {start_angle} to {end_angle}")
                sonar_sweep = self.get_sweep(start_angle, end_angle)
                if self.stream:
                    compressed_image = self.convert_to_ros_compressed_img(sonar_sweep)
                    self.sonar_image_publisher.publish(compressed_image)
            except KeyboardInterrupt:
                rospy.signal_shutdown("Shutting down sonar node.")

    def on_sonar_request(self, request):
        """ On a sonar request get the position of the object in the sweep

        Args:
            request (sweepGoal): Request message

        Returns:
            Nothing
        """
        if (request.distance_of_scan == -1):
            return
        self.set_new_range(request.distance_of_scan)

        left_gradians = sonar_utils.degrees_to_centered_gradians(request.start_angle)
        right_gradians = sonar_utils.degrees_to_centered_gradians(request.end_angle)

        rospy.loginfo(f"starting sweep from {left_gradians} to {right_gradians}")
        object_pose, sonar_sweep = self.get_xy_of_object_in_sweep(left_gradians, right_gradians)

        if self.stream:
            sonar_image = self.convert_to_ros_compressed_img(sonar_sweep)
            self.sonar_image_publisher.publish(sonar_image)

        self.pub_response.publish(object_pose)

    def run(self):
        """
        Main loop of the node
        """
        # If debug mode is on, do constant sweeps within range
        if self.debug:
            self.status_publisher.publish("Sonar running")
            self.constant_sweep(self.CONSTANT_SWEEP_START, self.CONSTANT_SWEEP_END, self.DEFAULT_RANGE)
        else:
            rospy.Subscriber(self.SONAR_REQUEST_TOPIC, SonarSweepRequest, self.on_sonar_request)
            rospy.loginfo("starting sonar status...")

            # Publish status for sensor check
            rate = rospy.Rate(20)
            while not rospy.is_shutdown():
                self.status_publisher.publish("Sonar running")
                rate.sleep()


if __name__ == '__main__':
    try:
        Sonar().run()
    except rospy.ROSInterruptException:
        pass
