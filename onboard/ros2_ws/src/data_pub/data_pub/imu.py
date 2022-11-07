#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import serial.tools.list_ports as list_ports
import traceback

from sensor_msgs.msg import Imu, MagneticField
from tf_transformations import quaternion_multiply


class IMURawPublisher(Node):

    IMU_DEST_TOPIC_QUAT = 'sensors/imu/imu'
    IMU_DEST_TOPIC_MAG = 'sensors/imu/mag'

    FTDI_STR = 'FT1WDFQ2'
    BAUDRATE = 115200
    NODE_NAME = 'imu_pub'
    LINE_DELIM = b','
    RETRY_PERIOD = 0.1
    RUN_LOOP_RATE = 20  # TODO: Determine if this rate is correct. Was previously in a while loop with no sleep call

    def __init__(self):
        super().__init__(self.NODE_NAME)
        self._pub_imu = self.create_publisher(Imu, self.IMU_DEST_TOPIC_QUAT, 50)
        self._pub_mag = self.create_publisher(MagneticField, self.IMU_DEST_TOPIC_MAG, 50)

        self._current_imu_msg = Imu()
        self._current_mag_msg = MagneticField()

        self._serial_port = None
        self._serial = None

        self.connection_timer = self.create_timer(self.RETRY_PERIOD, self._connect)
        self.run_timer = self.create_timer(1/self.RUN_LOOP_RATE, self._run)
        self.run_timer.cancel()

    def _connect(self):
        try:
            self._serial_port = next(list_ports.grep(self.FTDI_STR)).device
            self._serial = serial.Serial(self._serial_port, self.BAUDRATE,
                                         timeout=None, write_timeout=None,
                                         bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                                         stopbits=serial.STOPBITS_ONE)
            self.connection_timer.cancel()
            self.run_timer.reset()
        except StopIteration:
            self.get_logger().error(f"IMU not found, trying again in {self.RETRY_PERIOD} seconds.")

    def _run(self):
        try:
            line = self._serial.read_until()
            items = self._extract_line(line)
            if items[0] == b"$VNQMR":
                self._parse_orient(items)
                self._parse_accel(items)
                self._parse_angvel(items)
                self._parse_mag(items)
                self._publish_current_msg()
        except Exception:
            self.get_logger().error("Error in reading and extracting information. Reconnecting.")
            self.get_logger().error(traceback.format_exc())
            self._serial.close()
            self._serial = None
            self._serial_port = None
            self.run_timer.cancel()
            self.connection_timer.reset()

    def _extract_line(self, line):
        return line.split(self.LINE_DELIM)

    def _parse_orient(self, items):
        untransformed_orient = [float(items[1]), float(items[2]), float(items[3]), float(items[4])]
        # Transform quaternion from NED to ENU coordinates
        updated_quat = quaternion_multiply([0.707, 0.707, 0, 0], untransformed_orient)

        self._current_imu_msg.orientation.x = updated_quat[0]
        self._current_imu_msg.orientation.y = updated_quat[1]
        self._current_imu_msg.orientation.z = updated_quat[2]
        self._current_imu_msg.orientation.w = updated_quat[3]

    def _parse_accel(self, items):
        self._current_imu_msg.linear_acceleration.x = float(items[8])
        self._current_imu_msg.linear_acceleration.y = float(items[9])
        self._current_imu_msg.linear_acceleration.z = float(items[10])

    def _parse_angvel(self, items):
        self._current_imu_msg.angular_velocity.x = float(items[11])
        self._current_imu_msg.angular_velocity.y = float(items[12])
        items[13] = items[13][0:10]
        self._current_imu_msg.angular_velocity.z = float(items[13])

    def _parse_mag(self, items):
        self._current_mag_msg.magnetic_field.x = float(items[5])
        self._current_mag_msg.magnetic_field.y = float(items[6])
        self._current_mag_msg.magnetic_field.z = float(items[7])

    def _publish_current_msg(self):
        self._current_imu_msg.header.stamp = self.get_clock().now().to_msg()
        self._current_mag_msg.header.stamp = self.get_clock().now().to_msg()
        self._current_imu_msg.header.frame_id = "imu_link"
        self._current_mag_msg.header.frame_id = "imu_link"

        self._pub_imu.publish(self._current_imu_msg)
        self._current_imu_msg = Imu()
        self._pub_mag.publish(self._current_mag_msg)
        self._current_mag_msg = MagneticField()


def main(args=None):
    try:
        rclpy.init(args=args)
        imu = IMURawPublisher()
        rclpy.spin(imu)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        sys.exit(1)
    finally:
        imu.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
