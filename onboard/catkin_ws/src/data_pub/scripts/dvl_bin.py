#!/usr/bin/env python3

import serial
import serial.tools.list_ports as list_ports
import rospy
import yaml
import resource_retriever as rr
import traceback
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

class DvlPublisher:
    FTDI_FILE_PATH = 'package://data_pub/config/dvl_ftdi.yaml'

    BAUDRATE = 115200
    TOPIC_NAME = 'sensors/dvl/odom'
    NODE_NAME = 'dvl_publisher'

    def __init__(self):
        with open(rr.get_filename(self.FTDI_FILE_PATH, use_protocol=False)) as f:
            self._ftdi_strings = yaml.safe_load(f)

        self._pub = rospy.Publisher(self.TOPIC_NAME, Odometry, queue_size=10)

        self._current_msg = Odometry()

        self._serial_port = None
        self._serial = None

    def connect(self):
        while self._serial_port is None and not rospy.is_shutdown():
            try:
                self._serial_port = next(list_ports.grep('|'.join(self._ftdi_strings))).device
                self._serial = serial.Serial(self._serial_port, self.BAUDRATE,
                                             timeout=0.1, write_timeout=1.0,
                                             bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                                             stopbits=serial.STOPBITS_ONE)
            except StopIteration:
                rospy.logerr("DVL not found, trying again in 0.1 seconds.")
                rospy.sleep(0.1)

    def run(self):
        rospy.init_node(self.NODE_NAME)
        self.connect()

        while not rospy.is_shutdown():
            try:
                line = self._serial.readline().decode('utf-8')
                self._parse_line(line)
                if self._check_velocities():
                    self._publish_current_msg()
            except Exception:
                rospy.logerr("Error in reading and extracting information. Reconnecting.")
                rospy.logerr(traceback.format_exc())
                self._serial.close()
                self._serial = None
                self._serial_port = None
                self.connect()

    def _parse_line(self, line):
        """Parses binary data from DVL according to the #DP4 format. Check
           manual page 229 for more information.
        """
        bytes = [line[i:i+2] for i in range(0, len(line), 2)]
        
        #parse binary data
        if(bytes[0] != "7D" or bytes[0] != "7E"):
            rospy.logerr("Error in parsing binary DVL data: Bad Headder")
            return

        self.DATA_ID = int(bytes[0], 16)
        self.DATA_STRUCT = int(bytes[1], 16)
        self.NUM_BYTES = int(bytes[3] + bytes[2], 16)
        self.SYS_CONFIG = int(bytes[4], 16)
        self.X_VEL = self._signed(int(bytes[6] + bytes[5], 16))
        self.Y_VEL = self._signed(int(bytes[8] + bytes[7], 16)) 
        self.Z_VEL = self._signed(int(bytes[10] + bytes[9], 16))
        self.E_VEL = self._signed(int(bytes[12] + bytes[11], 16))
        self.BM1 = int(bytes[14] + bytes[13], 16)
        self.BM2 = int(bytes[16] + bytes[15], 16)
        self.BM3 = int(bytes[18] + bytes[17], 16)
        self.BM4 = int(bytes[20] + bytes[19], 16)
        self.BOTTOM_STATUS = int(bytes[21], 16)
        self.VEL_1 = self._signed(int(bytes[23] + bytes[22], 16))
        self.VEL_2 = self._signed(int(bytes[25] + bytes[24], 16))
        self.VEL_3 = self._signed(int(bytes[27] + bytes[26], 16))
        self.VEL_4 = self._signed(int(bytes[29] + bytes[28], 16))
        self.REF_LAYER_START = int(bytes[31] + bytes[30], 16)
        self.REF_LAYER_END = int(bytes[33] + bytes[32], 16)
        self.REF_LAYER_STATUS = int(bytes[34], 16)
        self.TOFP_HOUR = int(bytes[35], 16)
        self.TOFP_MIN = int(bytes[36], 16)
        self.TOFP_SEC = int(bytes[37], 16)
        self.TOFP_HUNDRETH = int(bytes[38], 16)
        self.LEAK_SENSOR = int(bytes[40] + bytes[39], 16)
        self.SPEED_OF_SOUND = int(bytes[42] + bytes[41], 16)
        self.TEMPERATURE = int(bytes[44] + bytes[43], 16)
        self.CHECKSUM = int(bytes[46] + bytes[45], 16)

    def _signed(i):
        """2's complement
        """
        if i >= 32768:
            return i - 65536
        else:
            return i
        
    def _check_velocities(self):
        """Check if the velocities are good
        """
        if(self.X_VEL == -32768 or self.Y_VEL == -32768 or self.Z_VEL == -32768):
            return False
        return True

    def _publish_current_msg(self):
        """Publish the current DVL message (velocity only)
        """
        current_time = rospy.Time.now()
        self._current_msg.header.stamp = current_time
        self._current_msg.header.frame_id = 'odom'

        vx = np.float64(self.X_VEL) / 1000
        vy = np.float64(self.Y_VEL) / 1000
        vz = np.float64(self.Z_VEL) / 1000

        self._current_msg.pose.pose = Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1))
        self._current_msg.child_frame_id = "dvl_link"
 
        self._current_msg.twist.twist = Twist(Vector3(vx, vy, vz), Vector3(0, 0, 0))
        self._current_msg.twist.covariance[0] = 0.01
        self._current_msg.twist.covariance[7] = 0.01
        self._current_msg.twist.covariance[14] = 0.01

        self._pub.publish(self._current_msg)


if __name__ == '__main__':
    try:
        DvlPublisher().run()
    except rospy.ROSInterruptException:
        pass
