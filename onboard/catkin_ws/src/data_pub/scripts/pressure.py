#!/usr/bin/env python3

import rospy
import serial
import serial.tools.list_ports as list_ports
import traceback

from sensor_msgs.msg import FluidPressure, Temperature


class Bar30RawPublisher:

    BAR30_DEST_TOPIC_PRESSURE = "sensors/bar30/pressure"
    BAR30_DEST_TOPIC_TEMP = "sensors/bar30/temp"

    FTDI_STR = ""
    BAUDRATE = 9600
    NODE_NAME = "bar30_pub"
    DATA_START = b"$BAR30\r"
    DATA_DELIM = b"@"
    LINE_DELIM = b"\n"

    def __init__(self):
        self._pub_pressure = rospy.Publisher(
            self.BAR30_DEST_TOPIC_PRESSURE, FluidPressure, queue_size=50
        )
        self._pub_temp = rospy.Publisher(
            self.BAR30_DEST_TOPIC_TEMP, Temperature, queue_size=50
        )

        self._current_pressure_msg = FluidPressure()
        self._current_temp_msg = Temperature()

        self._serial_port = None
        self._serial = None

    def connect(self):
        while self._serial_port is None and not rospy.is_shutdown():
            try:
                self._serial_port = next(list_ports.grep(self.FTDI_STR)).device
                self._serial = serial.Serial(
                    self._serial_port,
                    self.BAUDRATE,
                    timeout=None,
                    write_timeout=None,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                )
            except StopIteration:
                rospy.logerr("Bar30 not found, trying again in 0.1 seconds.")
                rospy.sleep(0.1)

    def run(self):
        rospy.init_node(self.NODE_NAME)
        self.connect()
        while not rospy.is_shutdown():
            try:
                line = self._serial.read_until(self.DATA_DELIM)
                items = self._extract_line(line)
                if items[0] == self.DATA_START:
                    self._parse_pressure(items[1])
                    self._parse_temp(items[2])
                    self._publish_current_msg()
            except Exception:
                rospy.logerr(
                    "Error in reading and extracting information. Reconnecting."
                )
                rospy.logerr(traceback.format_exc())
                self._serial.close()
                self._serial = None
                self._serial_port = None
                self.connect()

    def _extract_line(self, line):
        return line.split(self.LINE_DELIM)

    def _parse_pressure(self, item):
        item = item.decode("utf-8").split(":")
        data = item[-1].rstrip()
        self._current_pressure_msg.fluid_pressure = float(data[1:])

    def _parse_temp(self, item):
        item = item.decode("utf-8").split(":")
        data = item[-1].rstrip()
        self._current_temp_msg.temperature = float(data[1:])

    def _publish_current_msg(self):
        self._current_pressure_msg.header.stamp = rospy.Time.now()
        self._current_temp_msg.header.stamp = rospy.Time.now()

        self._pub_pressure.publish(self._current_pressure_msg)
        self._current_pressure_msg = FluidPressure()
        self._pub_temp.publish(self._current_temp_msg)
        self._current_temp_msg = Temperature()


if __name__ == "__main__":
    try:
        Bar30RawPublisher().run()
    except rospy.ROSInterruptException:
        pass
