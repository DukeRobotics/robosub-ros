
import rospy
import numpy as np

from std_msgs.msg import Float32, Int32
from custom_msgs.msg import ThrusterAllocs

CONTROL_EFFORTS_TOPIC = "controls/thruster_allocs"
VOLTAGE_TOPIC = "controls/voltage"
PWM_PUBLISHER_TOPIC = "offboard_comms/pwm"

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
        pass

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

    # This method takes in voltage and desired control effort and outputs pwm via the fitted polynomial model
    def polynomial(self):
        pass

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    ThrusterConverter().run()
