#!/usr/bin/env python3

import rospy
import numpy as np

from bisect import bisect_left

from std_msgs.msg import Float32, Int32
from custom_msgs.msg import ThrusterAllocs

CONTROL_EFFORTS_TOPIC = "controls/thruster_allocs"
VOLTAGE_TOPIC = "controls/voltage"  # This could change
PWM_PUBLISHER_TOPIC = "offboard_comms/pwm"

COEFFICIENTS = [-0.72598292, 17.51636272, -1.96635442, 341.37137439, 17.68477542,
                -0.61271068, -42.99373034, 1.3453298]


class ThrusterConverter:

    def __init__(self):

        # Instantiate new thruster converter node
        rospy.init_node("thrusters", anonymous=True)
        # Read roslaunch params
        self.interpolation_mode = rospy.get_param("~interpolation")

        # Default voltage is 15.5
        self.voltage = 15.6

        # Pre-load look-up tables
        self.load_lookup()

        # Subscriber to read control efforts \in [-1.0, 1.0]
        # Currently we only publish a desired PWM output when we receive a control effort
        self.control_effort_subscriber = rospy.Subscriber(CONTROL_EFFORTS_TOPIC,
                                                          ThrusterAllocs, self.convert_and_publish)
        # Subscriber to read voltages; expected range is \in [14.0, 16.2], though it can handle up to 18.0v
        self.voltage_subscriber = rospy.Subscriber(VOLTAGE_TOPIC, Float32, self.update_voltage)
        # TODO: Check to see if it's better to constantly publish the last read control effort
        self.pwm_publisher = rospy.Publisher(PWM_PUBLISHER_TOPIC, Int32, queue_size=1)

    # This method is the callback for returning the thruster pwm output
    # depending on voltage and desired force
    def convert_and_publish(self, desired_effort):
        # Check which model we are using: discrete (lookup table) or continuous (polynomial)
        if self.interpolation_mode == "discrete":
            self.pwm_publisher.publish(self.lookup(desired_effort, self.voltage))
        else:
            # continuous, or polynomial mode
            self.pwm_publisher.publish(self.polynomial(desired_effort, self.voltage))

    # This method is the callback for updating the latest voltage read
    def update_voltage(self, voltage):
        # Check to see if voltage is within expected bounds
        if voltage >= 16.2:
            # If greater than the maximum expected amount, round down to 16.2
            self.voltage = 16.2
        elif voltage < 14.0:
            # If lower than minimum expected amount, round up to 14.0
            self.voltage = 14.0
        else:
            self.voltage = voltage

    # This method pre-loads the lookup tables under the dictionary voltage_efforts_to_pwm
    def load_lookup(self):
        # Dictionary for the lookup table
        self.voltage_efforts_to_pwm = dict()
        # Load all computed power curves from 14.0v to 18.0v
        for voltage in range(14.0, 18.1, 0.1):
            # Read .npy file to load pre-calculated linear interpolation data
            file_path = "./data/" + str(voltage) + "_interpolated.npy"
            loaded_np_array = np.load(file_path)
            pwm, force = loaded_np_array[:, 0], loaded_np_array[:, 1]
            # self.voltage_efforts_to_pwm[15.5] = [forces array for 15.5, pwm array for 15.5]
            self.voltage_efforts_to_pwm[voltage] = [force, pwm]

    # This method takes in voltage and desired control effort and outputs pwn
    # via the linearly interpolated lookup tables
    def lookup(self, voltage, force):
        # Round voltage to the nearest tenth decimal place
        voltage = round(voltage, 1)
        # Efficiently look up the requested index for a given query (voltage, force) in table via binary search
        # We use binary search to find the left insertion position of force in the associated force array
        # Note that "force" is always in sorted increasing order, so we can use an array bisection algorithm
        left_index = bisect_left(self.voltage_efforts_to_pwm[voltage][0], force)
        return self.voltage_efforts_to_pwm[voltage][1][left_index]

    # This method takes in voltage and desired control effort and outputs pwm
    # via the fitted polynomial model
    def polynomial(self, voltage, force):
        terms = [voltage ** 3, voltage ** 2, force ** 3, force ** 2, voltage * force,
                 (voltage ** 2) * force, voltage * (force ** 2), (voltage ** 2) * (force ** 2)]
        return sum([COEFFICIENTS[i] * terms[i] for i in range(len(terms))])

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    ThrusterConverter().run()
