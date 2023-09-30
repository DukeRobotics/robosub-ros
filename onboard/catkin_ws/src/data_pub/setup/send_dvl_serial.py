#!/usr/bin/env python3

import serial
import serial.tools.list_ports as list_ports
import rospy
import yaml
import resource_retriever as rr
import time
import binascii

FTDI_FILE_PATH = 'package://data_pub/config/dvl_ftdi.yaml'
BAUDRATE = 115200

serial_port = None
myserial = None
ftdi_strings = None

def connect():
    global serial_port, myserial, ftdi_strings
    while serial_port is None and not rospy.is_shutdown():
        try:
            serial_port = next(list_ports.grep('|'.join(ftdi_strings))).device
            myserial = serial.Serial(serial_port, BAUDRATE,
                                            timeout=0.1, write_timeout=1.0,
                                            bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                                            stopbits=serial.STOPBITS_ONE)
        except StopIteration:
            rospy.logerr("DVL not found, trying again in 0.1 seconds.")
            rospy.sleep(0.1)

def main():
    global serial_port, myserial, ftdi_strings

    with open(rr.get_filename(FTDI_FILE_PATH, use_protocol=False)) as f:
        ftdi_strings = yaml.safe_load(f)

    connect()

    allout = False

    try:
        while True:
            myserial.flush()
            cmd = input("Enter command: ")
            if cmd == "":
                continue
            cmd = cmd + "\n"
            if cmd == "break\n":
                cmd = "þ\n"
                allout = True
            myserial.write(cmd.encode('utf-8'))
            line = myserial.readline().decode('utf-8')
            itr = 0
            while not cmd[:-1] in line:
                line = myserial.readline().decode('utf-8')
                itr += 1
                if itr > 150:
                    # print("bad execution")
                    myserial.write(cmd.encode('utf-8'))
                    line = myserial.readline().decode('utf-8')
                    itr = 0
                    continue
            itr = 0
            while True:
                if allout:
                    print(line[:-1])
                    line = myserial.readline().decode('utf-8')
                    continue
                itr += 1
                if ">" in line:
                    break
                if itr > 150:
                    break
                if line[0] != ":":
                    print(line[:-1])
                line = myserial.readline().decode('utf-8')
            myserial.flush()
    except KeyboardInterrupt:
        print("\nKeyboard interrupt received. Exiting.")
        myserial.close()
        exit(0)

if __name__ == "__main__":
    main()

#things changed in the DVL
#

#Test
#EU - up down mode
