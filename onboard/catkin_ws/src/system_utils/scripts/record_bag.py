#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64

import subprocess
import signal
import os
from datetime import datetime


class RecordBag:

    # Duration of time to wait for a voltage message before stopping recording
    TIMEOUT_DURATION = rospy.Duration(5)

    def __init__(self):
        # Initialize variables
        self.process = None

        # Initialize last message time to current time
        self.last_msg_time = rospy.Time.now()

        # Subscribe to the voltage topic
        rospy.Subscriber("/sensors/voltage", Float64, self.voltage_callback)

        # Create a timer to check for timeout; calls check_timeout every second
        self.timer = rospy.Timer(rospy.Duration(1), self.check_timeout)

        rospy.loginfo("Record bag node started.")

    def voltage_callback(self, data: Float64):
        """
        Callback function for the voltage topic. If the voltage is above 5V and the node is not already recording,
        start recording. If the voltage drops below 5V and the node is currently recording, stop recording.

        Args:
            data: The voltage value published to the topic.
        """

        # Update last received message time
        self.last_msg_time = rospy.Time.now()

        # If voltage is below 5V and the node is currently recording, stop recording
        if data.data < 5:
            if self.process:
                rospy.loginfo("Voltage is below 5V. Stopping recording due to low voltage.")
                self.stop_recording()
                self.shutdown_node()

        # If voltage is above 5V and the node is not currently recording, start recording
        else:
            if self.process is None:
                self.start_recording()

    def start_recording(self):
        """
        Start recording all topics to a bag file by executing the `rosbag record` command in the shell. The bag file is
        saved in the bag_files directory in the robosub-ros package. The file name is the current date and time in a
        human-readable format.
        """

        # Get the current time in seconds since the Unix epoch
        current_time_sec = rospy.Time.now().to_sec()

        # Convert to a human-readable format
        human_readable_time = datetime.fromtimestamp(current_time_sec).strftime('%Y.%m.%d_%I-%M-%S_%p')

        # Start recording all topics to a bag file
        command = f"rosbag record -a -O /root/dev/robosub-ros/bag_files/{human_readable_time}.bag"
        self.process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, shell=True,
                                        preexec_fn=os.setsid)

        rospy.loginfo("Started recording all topics.")

    def stop_recording(self):
        """
        Stop recording the bag file, if it is currently recording.
        """
        if self.process:
            os.killpg(os.getpgid(self.process.pid), signal.SIGINT)
            self.process = None
            rospy.loginfo("Recording stopped.")

    def check_timeout(self, event):
        """
        Check if the last voltage message received was more than TIMEOUT_DURATION ago. If so, and if the node is
        currently recording, stop recording and shutdown the node.

        Args:
            event: The timer event that triggered this function.
        """
        current_time = rospy.Time.now()
        if (current_time - self.last_msg_time) > self.TIMEOUT_DURATION:
            if self.process is not None:
                rospy.loginfo(f"No voltage messages received for {self.TIMEOUT_DURATION.to_sec()} seconds. "
                              f"Stopping recording.")
                self.stop_recording()
                self.shutdown_node()

    def shutdown_node(self):
        """
        Shutdown the node and stop recording.
        """
        rospy.signal_shutdown("Stopping node due to recording stop.")

    def run(self):
        """
        Run the node. The node will wait for voltage messages to start recording. If the node is shutdown, it will stop
        recording.
        """
        rospy.spin()

        if self.process:
            rospy.loginfo("Shutting down. Stopping recording.")
            self.stop_recording()


if __name__ == '__main__':

    rospy.init_node('record_bag', anonymous=True)

    # Get enable_recording launch param
    enable_recording = rospy.get_param("~enable_recording", True)

    # If recording is disabled, exit
    if enable_recording:
        recorder = RecordBag()

        try:
            recorder.run()
        except rospy.ROSInterruptException:
            pass
    else:
        rospy.loginfo("Recording is disabled.")
        rospy.signal_shutdown("Recording is disabled.")
