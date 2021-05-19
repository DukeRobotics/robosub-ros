#!/usr/bin/env python3

import rospy
import serial
import serial.tools.list_ports as list_ports

from sensor_msgs.msg import Imu, MagneticField
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class IMURawPublisher:

    IMU_DEST_TOPIC_QUAT = 'sensors/imu/imu'
    IMU_DEST_TOPIC_MAG = 'sensors/imu/mag'

    FTDI_STR = 'FT1WDFQ2'
    BAUDRATE = 115200
    NODE_NAME = 'imu_pub'
    LINE_DELIM = ','

    def __init__(self):
        self._pub_imu = rospy.Publisher(self.IMU_DEST_TOPIC_QUAT, Imu, queue_size=50)
        self._pub_mag = rospy.Publisher(self.IMU_DEST_TOPIC_MAG, MagneticField, queue_size=50)

        self._current_imu_msg = Imu()
        self._current_mag_msg = MagneticField()

        self._serial_port = None
        self._serial = None

    def run(self):
        rospy.init_node(self.NODE_NAME)

        while self._serial_port is None and not rospy.is_shutdown():
            try:
                self._serial_port = next(list_ports.grep(self.FTDI_STR)).device
                self._serial = serial.Serial(self._serial_port, self.BAUDRATE,
                                             timeout=None, write_timeout=None,
                                             bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                                             stopbits=serial.STOPBITS_ONE)
            except StopIteration:
                rospy.logerr("IMU not found, trying again in 1 second.")
                rospy.sleep(1)

        while not rospy.is_shutdown():
            line = self._serial.read_until()
            items = self._extract_line(line)  # array of line
            if items[0] == "$VNQMR":
                rospy.loginfo(line)
                self._parse_orient(items)
                self._parse_accel(items)
                self._parse_angvel(items)
                self._parse_mag(items)
                self._publish_current_msg()

    def _extract_line(self, line):
        return line.split(self.LINE_DELIM)

    def _parse_orient(self, items):
        r, p, y = euler_from_quaternion([float(items[1]), float(items[2]), float(items[3]), float(items[4])])
        p = -p
        y = -y
        updated_quat = quaternion_from_euler(r, p, y)

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
        self._current_imu_msg.header.stamp = rospy.Time.now()
        self._current_mag_msg.header.stamp = rospy.Time.now()
        self._current_imu_msg.header.frame_id = "imu_link"
        self._current_mag_msg.header.frame_id = "imu_link"

        self._pub_imu.publish(self._current_imu_msg)
        self._current_imu_msg = Imu()
        self._pub_mag.publish(self._current_mag_msg)
        self._current_mag_msg = MagneticField()


if __name__ == '__main__':
    try:
        IMURawPublisher().run()
    except rospy.ROSInterruptException:
        pass
