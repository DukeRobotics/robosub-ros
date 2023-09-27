#!/usr/bin/env python3

import serial
import serial.tools.list_ports as list_ports
import rospy
import yaml
import resource_retriever as rr
import traceback
import asyncio


FTDI_FILE_PATH = 'package://data_pub/config/dvl_ftdi.yaml'
BAUDRATE = 115200

serial_port = None
myserial = None

def connect():
    global serial_port, myserial
    while serial_port is None and not rospy.is_shutdown():
        try:
            serial_port = next(list_ports.grep('|'.join(_ftdi_strings))).device
            myserial = serial.Serial(serial_port, BAUDRATE,
                                            timeout=0.1, write_timeout=1.0,
                                            bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                                            stopbits=serial.STOPBITS_ONE)
        except StopIteration:
            rospy.logerr("DVL not found, trying again in 0.1 seconds.")
            rospy.sleep(0.1)

with open(rr.get_filename(FTDI_FILE_PATH, use_protocol=False)) as f:
    _ftdi_strings = yaml.safe_load(f)

connect()


def main():
    global serial_port, myserial
    try:
        while True:
            line = myserial.readline().decode('utf-8')
            if line:
                print(line[:-1])
    except KeyboardInterrupt:
        print("\nKeyboard interrupt received. Exiting.")
        myserial.close()
        exit(0)

if __name__ == "__main__":
    main()

#things changed in the DVL
#


