#!/usr/bin/env python3

import argparse
import resource_retriever as rr
import serial.tools.list_ports as list_ports
import yaml

# Get the path to the YAML file containing the FTDI strings of all arduinos
FTDI_FILE_PATH = 'package://offboard_comms/config/arduino_ftdi.yaml'

# Read FTDI strings of all arduinos
with open(rr.get_filename(FTDI_FILE_PATH, use_protocol=False)) as f:
    FTDI_STRINGS = yaml.safe_load(f)
    ARDUINO_NAMES = list(FTDI_STRINGS.keys())

# Get the port of the requested arduino
def get_arduino_port(arduino_name):
    try:
        ftdi_string = FTDI_STRINGS[arduino_name]
        return next(list_ports.grep(ftdi_string)).device
    except StopIteration:
        raise Exception(f"Could not find {arduino_name} arduino.")

def main():
    # Parse arguments
    parser = argparse.ArgumentParser(description="Get pressure or thruster arduino port.")
    parser.add_argument("arduino_name", choices=ARDUINO_NAMES, help=f"Specify one of {ARDUINO_NAMES}.")

    args = parser.parse_args()

    # Check if the arduino name is valid
    if args.arduino_name not in ARDUINO_NAMES:
        print(f"Error: Unknown arduino name '{args.arduino_name}'. It must be one of {ARDUINO_NAMES}.")
        return

    # Print the port of the requested arduino
    port = get_arduino_port(args.arduino_name)
    print(port, end='')

if __name__ == "__main__":
    main()
