#!/usr/bin/env python3

from brping import Ping360
import numpy as np
import sonar_utils
import matplotlib.pyplot as plt
import cv2
#from tf import TransformListener
from geometry_msgs.msg import Pose

class Sonar:
    """Class to interface with the Sonar device.
    """

    SERIAL_PORT_NAME = "/dev/ttyUSB2"  # PORT of the salea is ttyUSB2 for testing
    BAUD_RATE = 2000000  # hz
    SAMPLE_PERIOD_TICK_DURATION = 25e-9  # s
    SPEED_OF_SOUND_IN_WATER = 1480  # m/s
    FILTER_INDEX = 100 #number of values to filter TODO figure out where the noise starts

    def __init__(self, range, number_of_samples=1200, serial_port_name=SERIAL_PORT_NAME, baud_rate=BAUD_RATE):
        self.ping360 = Ping360()
        self.ping360.connect_serial(serial_port_name, baud_rate)  # TODO: Add try except for connecting to device
        #self.ping360.connect_udp(self.ETHERNET_PORT_NAME)
        self.ping360.initialize()
        period_and_duration = self.range_to_period_and_duration(range)

        self.number_of_samples = number_of_samples
        self.ping360.set_number_of_samples(number_of_samples)
        self.sample_period = period_and_duration[0]
        self.ping360.set_sample_period(self.sample_period)
        self.transmit_duration = period_and_duration[1]
        self.ping360.set_transmit_duration(self.transmit_duration)

        #self.listener = TransformListener()

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

    def angle_to_radian(self, angle):
        """Converts gradians to degrees 

        Returns:
            angle in degrees
        """
        return (angle-200)*np.pi/200

    def to_robot_position(self, angle, index):
        """Converts a point in sonar space a robot global position

        Returns:
            (coordinate)
        """
        #Need to change the static transform for where the sonar is on the robot
        x_pos = self.get_distance_of_sample(index) * np.cos(self.angle_to_radian(angle))
        y_pos = self.get_distance_of_sample(index) * np.sin(self.angle_to_radian(angle))
        pos_of_point = Pose()
        pos_of_point.position.x = x_pos
        pos_of_point.position.y = y_pos
        pos_of_point.position.z = 0 #z cord isnt 0 as gate is a line 
        pos_of_point.orientation.x = 0
        pos_of_point.orientation.y = 0
        pos_of_point.orientation.z = 0
        pos_of_point.orientation.w = 1

        global_pose = sonar_utils.transform_pose(self.listener, "sonar_link", "odom", pos_of_point)

        return global_pose

    def find_gate_posts(img): 
        greyscale_image = cv2.cvtColor(img.astype(np.uint8), cv2.COLOR_GRAY2BGR)
        cm_image = cv2.applyColorMap(greyscale_image, cv2.COLORMAP_VIRIDIS)

        cm_copy_image = cm_image
        cv2.copyTo(cm_image, cm_copy_image)
        cm_image = cv2.medianBlur(cm_image,5) # blur image

        lower_color_bounds = (40,80,0) # filter out lower values (ie blue)
        upper_color_bounds = (230,250,255) #filter out too high values
        mask = cv2.inRange(cm_image,lower_color_bounds,upper_color_bounds)

        cm_circles = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[-2]
        cm_circles = sorted(cm_circles, key=cv2.contourArea, reverse=True)
        cm_circles = list(filter(lambda x: (cv2.contourArea(x) > 200), cm_circles)) 
        cm_circles = list(filter(lambda x: (cv2.arcLength(x, True)**2/(4*math.pi*cv2.contourArea(x)) < 5.4), cm_circles)) 

        #return if not both goal posts found 
        if(len(cm_circles) < 1):
            print("not enough circles found")
            return None

        filtered_circles = cm_circles[0:2]

        circle_positions = []
        for circle in filtered_circles:  #find center of circle code
            M = cv2.moments(circle)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            circle_positions.append((cX,cY))

        return circle_positions


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
    #sweep_data = sonar.sweep_biggest_byte(100, 300)  #180deg in front
    #print(f"Distance to object: {sonar.get_distance_of_sample(sweep_data[0])} | Angle: {sweep_data[2]}")

    for i in range(100, 300):
        data = sonar.request_data_at_angle(i).data
        split_bytes = [data[i:i+1] for i in range(len(data))]
        split_bytes = split_bytes[100:]
        byte_from_int = int.from_bytes(split_bytes[0], "big")
        intarray = np.array([byte_from_int])
        for i in range(len(split_bytes) -1):
            byte_from_int = int.from_bytes(split_bytes[i+1], "big")
            intarray = np.append(intarray, [byte_from_int])
        if(i == 150):
            sonar_matrix = np.asarray(intarray)
        else:
            sonar_matrix = np.vstack((sonar_matrix, intarray))
    plt.imsave('onboard\\catkin_ws\\src\\sonar\\scripts\\sampleData\\Sonar_Image_robot.jpeg', sonar_matrix)
    np.save('onboard\\catkin_ws\\src\\sonar\\scripts\\sampleData\\Sonar_Matrix_robot.npy', sonar_matrix)
    
    found_posts = sonar.find_gate_posts(sonar_matrix)
    print(found_posts)
    
