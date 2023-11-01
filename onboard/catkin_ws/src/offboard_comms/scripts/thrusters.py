#!/usr/bin/env python3

import rospy
import numpy as np

from bisect import bisect_left

from std_msgs.msg import Float32, Int32
from custom_msgs.msg import ThrusterAllocs, PWMAllocs

CONTROL_EFFORTS_TOPIC = "controls/thruster_allocs"
VOLTAGE_TOPIC = "controls/voltage"  # This could change
PWM_PUBLISHER_TOPIC = "offboard_comms/pwm"

# Coefficients for the fitted polynomial; terms of the polynomial are shown under function polynomial
COEFFICIENTS = [-7.65838439e-01,  1.84459825e+01, -2.01132736e+02, 7.42118554e+03,
                7.92882050e+01, -2.70347565e+00, -9.44255430e+02, 2.95486429e+01]


class ThrusterConverter:

    def __init__(self):

        # Instantiate new thruster converter node
        rospy.init_node("thrusters", anonymous=True)
        # Read roslaunch params
        self.interpolation_mode = rospy.get_param("~interpolation")

        # Default voltage is 15.6
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
        self.pwm_publisher = rospy.Publisher(PWM_PUBLISHER_TOPIC, PWMAllocs, queue_size=1)

    # This method is the callback for returning the thruster pwm output
    # depending on voltage and desired force
    def convert_and_publish(self, desired_efforts):
        # Check which model we are using: discrete (lookup table) or continuous (polynomial)
        if self.interpolation_mode == "discrete":
            pwm = np.zeros(8).astype(float)
            for i, desired_effort in enumerate(desired_efforts.allocs):
                pwm[i] = self.lookup(desired_effort, self.voltage)
            PWM_alloc = PWMAllocs()
            PWM_alloc.allocs = pwm
            self.pwm_publisher.publish(PWM_alloc)
        else:
            # continuous, or polynomial mode
            pwm = np.zeros(8).astype(float)
            for i, desired_effort in enumerate(desired_efforts.allocs):
                pwm[i] = self.polynomial(desired_effort, self.voltage)
            PWM_alloc = PWMAllocs()
            PWM_alloc.allocs = pwm
            self.pwm_publisher.publish(PWM_alloc)

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
        # Load power curve tables for 14.0v, 16.0v, and 18.0v
        all_14v, all_16v, all_18v = np.load("./data/14.npy"), np.load("./data/16.npy"), np.load("./data/18.npy")
        # Save data under appropriate voltage heading/key in voltage_efforts_to_pwm
        self.voltage_efforts_to_pwm[14.0] = [all_14v[:, 1], all_14v[:, 0]]
        self.voltage_efforts_to_pwm[16.0] = [all_16v[:, 1], all_16v[:, 0]]
        self.voltage_efforts_to_pwm[18.0] = [all_18v[:, 1], all_18v[:, 0]]

    # This method takes in voltage and desired control effort and outputs pwn
    # via the linearly interpolated lookup tables
    def lookup(self, voltage, force):
        # Round voltage to the nearest tenth decimal place
        voltage = round(voltage, 1)
        # Efficiently look up the requested index for a given query (voltage, force) in table via binary search
        # We use binary search to find the left insertion position of force in the associated force array
        # Note that "force" is always in sorted increasing order, so we can use an array bisection algorithm
        left_index = bisect_left(self.voltage_efforts_to_pwm[voltage][0], force)

        # Find interpolation range
        if 14.0 <= voltage and voltage <= 16.0:
            interpolated_pwm = self.interpolate(14.0, self.voltage_efforts_to_pwm[14.0][left_index],
                                                16.0, self.voltage_efforts_to_pwm[16.0][left_index], force)
            return interpolated_pwm
        else:
            interpolated_pwm = self.interpolate(16.0, self.voltage_efforts_to_pwm[16.0][left_index],
                                                18.0, self.voltage_efforts_to_pwm[18.0][left_index], force)
            return interpolated_pwm

    # Linear interpolation to process data
    def interpolate(self, x1, y1, x2, y2, x_interpolate):
        return y1 + ((y2 - y1) * (x_interpolate - x1))/(x2 - x1)

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
