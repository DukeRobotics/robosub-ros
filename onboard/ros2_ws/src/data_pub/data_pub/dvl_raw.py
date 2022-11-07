#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import serial.tools.list_ports as list_ports
import traceback

from custom_msgs.msg import DVLRaw


class DvlRawPublisher(Node):

    FTDI_STR = '7006fIP'
    BAUDRATE = 115200
    TOPIC_NAME = 'sensors/dvl/raw'
    NODE_NAME = 'dvl_raw_publisher'
    LINE_DELIM = ','
    RETRY_PERIOD = 0.1
    RUN_LOOP_RATE = 20  # TODO: Determine if this rate is correct. Was previously in a while loop with no sleep call

    def __init__(self):
        super().__init__(self.NODE_NAME)
        self._pub = self.create_publisher(DVLRaw, self.TOPIC_NAME, 10)
        self._current_msg = DVLRaw()

        self._serial_port = None
        self._serial = None

        self._dvl_line_parsers = {
            'SA': self._parse_SA,
            'TS': self._parse_TS,
            'BI': self._parse_BI,
            'BS': self._parse_BS,
            'BE': self._parse_BE,
            'BD': self._parse_BD
        }

        self.connection_timer = self.create_timer(self.RETRY_PERIOD, self._connect)
        self.run_timer = self.create_timer(1/self.RUN_LOOP_RATE, self._run)
        self.run_timer.cancel()

    def _connect(self):
        try:
            self._serial_port = next(list_ports.grep(self.FTDI_STR)).device
            self._serial = serial.serial(self._serial_port, self.baudrate,
                                         timeout=0.1, write_timeout=1.0,
                                         bytesize=serial.eightbits, parity=serial.parity_none,
                                         stopbits=serial.stopbits_one)
            self.connection_timer.cancel()
            self.run_timer.reset()
        except StopIteration:
            self.get_logger().error(f"DVL not found, trying again in {self.RETRY_PERIOD} seconds.")

    def _run(self):
        try:
            line = self._serial.readline().decode('utf-8')
            if line.strip() and line[0] == ':':
                self._parse_line(line)
        except Exception:
            self.get_logger().error("Error in reading and extracting information. Reconnecting.")
            self.get_logger().error(traceback.format_exc())
            self._serial.close()
            self._serial = None
            self._serial_port = None
            self.run_timer.cancel()
            self.connection_timer.reset()

    def _parse_line(self, line):
        data_type = line[1:3]
        self._dvl_line_parsers[data_type](self._clean_line(line))

    def _clean_line(self, line):
        return line[4:].replace('\r\n', '')

    def _parse_SA(self, line):
        fields = self._extract_floats(line, 0, None)
        self._current_msg.sa_roll = fields[0]
        self._current_msg.sa_pitch = fields[1]
        self._current_msg.sa_heading = fields[2]

    def _parse_TS(self, line):
        fields = self._extract_floats(line, 1, None)
        self._current_msg.ts_salinity = fields[0]
        self._current_msg.ts_temperature = fields[1]
        self._current_msg.ts_depth = fields[2]
        self._current_msg.ts_sound_speed = fields[3]
        self._current_msg.ts_built_in_test = int(fields[4])

    def _parse_BI(self, line):
        fields = self._extract_floats(line, 0, 4)
        self._current_msg.bi_x_axis = fields[0]
        self._current_msg.bi_y_axis = fields[1]
        self._current_msg.bi_z_axis = fields[2]
        self._current_msg.bi_error = fields[3]
        self._current_msg.bi_status = line.split(self.LINE_DELIM)[4]

    def _parse_BS(self, line):
        fields = self._extract_floats(line, 0, 3)
        self._current_msg.bs_transverse = fields[0]
        self._current_msg.bs_longitudinal = fields[1]
        self._current_msg.bs_normal = fields[2]
        self._current_msg.bs_status = line.split(self.LINE_DELIM)[3]

    def _parse_BE(self, line):
        fields = self._extract_floats(line, 0, 3)
        self._current_msg.be_east = fields[0]
        self._current_msg.be_north = fields[1]
        self._current_msg.be_upwards = fields[2]
        self._current_msg.be_status = line.split(self.LINE_DELIM)[3]

    def _parse_BD(self, line):
        fields = self._extract_floats(line, 0, None)
        self._current_msg.bd_east = fields[0]
        self._current_msg.bd_north = fields[1]
        self._current_msg.bd_upwards = fields[2]
        self._current_msg.bd_range = fields[3]
        self._current_msg.bd_time = fields[4]

        # BD type is the last message received, so publish
        self._publish_current_msg()

    def _extract_floats(self, num_string, start, stop):
        """Return a list of floats from a given string,
        using LINE_DELIM and going from start to stop
        """
        return [float(num) for num in num_string.split(self.LINE_DELIM)[start:stop]]

    def _publish_current_msg(self):
        """Publish the current DVL message and set the message to empty
        """
        self._pub.publish(self._current_msg)
        self._current_msg = DVLRaw()


def main(args=None):
    try:
        rclpy.init(args=args)
        dvl_raw = DvlRawPublisher()
        rclpy.spin(dvl_raw)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        sys.exit(1)
    finally:
        dvl_raw.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
