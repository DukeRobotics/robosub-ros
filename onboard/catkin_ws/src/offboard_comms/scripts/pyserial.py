#!/usr/bin/env python

import rospy
import serial
from custom_msgs.msg import PWMAllocs

class serial_thrusters:
    def __init__(self, baud_rate, timeout=1):
        """
        Constructor to initialize the serial connection.
        
        :param baud_rate: Baud rate for the connection (e.g., 9600, 115200)
        :param timeout: Timeout for the connection in seconds (default is 1 second)
        """
        # Get serial port
        self.device = rospy.get_param('~device', '/dev/default')  # '~device' ensures private namespace

        try:
            self.ser = serial.Serial(port=self.device, baudrate=baud_rate, timeout=timeout)
            print(f"Serial connection initialized on {self.device} at {baud_rate} baud.")
        except serial.SerialException as e:
            print(f"Error initializing serial connection: {e}")
            self.ser = None

        # Initialize the node
        rospy.init_node('/offboard/serial_thruster')

        # Create a subscriber
        thruster_pwm_sub = rospy.Subscriber('/offboard/pwm', PWMAllocs, self.send_message)

        # Register the shutdown hook
        rospy.on_shutdown(self.shutdown)

    def shutdown(self):
        """Clean up resources on shutdown."""
        rospy.loginfo("Shutting down Thrusters node...")
        if hasattr(self, 'serial_connection') and self.serial_connection.is_open:
            rospy.loginfo("Closing serial connection...")
            self.serial_connection.close()

    def send_message(self, message):
        """
        Sends a message over the serial connection.
        
        :param message: String message to send
        """
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(message.encode())  # Encode the message to bytes
                print(f"Sent: {message}")
            except Exception as e:
                print(f"Error sending message: {e}")
        else:
            print("Serial connection is not open. Cannot send message.")

if __name__ == '__main__':
    try:
        serial_thrusters(baud_rate=57600).run()
    except rospy.ROSInterruptException:
        pass