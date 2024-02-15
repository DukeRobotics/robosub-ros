#!/usr/bin/env python3

import argparse
import os
import serial.tools.list_ports as list_ports
import subprocess
from typing import List, Union, Sequence, Optional
import yaml

import resource_retriever as rr

# Template for the path to the offboard_comms package
OFFBOARD_COMMS_PATH_TEMPLATE = 'package://offboard_comms/{subpath}'

# Path to the arduino YAML file
CONFIG_YAML_PATH = OFFBOARD_COMMS_PATH_TEMPLATE.format(subpath=f'config/{os.getenv("ROBOT_NAME", "oogway")}.yaml')

# Commands to install the ros_lib library for Arduino
ROS_LIB_INSTALL_COMMANDS = [
    'rm -rf ros_lib',
    'rm -f ros_lib.zip',
    'rosrun rosserial_arduino make_libraries.py .',
    'zip -r ros_lib.zip ros_lib',
    'arduino-cli lib install --zip-path ros_lib.zip',
    'rm -f ros_lib.zip',
    'rm -rf ros_lib',
]

# Environment variables to update when installing the ros_lib library for Arduino
ROS_LIB_ENV_UPDATES = {
    'ARDUINO_LIBRARY_ENABLE_UNSAFE_INSTALL': 'true'
}

# Path to run ros_lib installation commands at
ROS_LIB_INSTALL_PATH = rr.get_filename(OFFBOARD_COMMS_PATH_TEMPLATE.format(subpath=''), use_protocol=False)

# Command templates for Arduino CLI
ARDUINO_CORE_INSTALL_COMMAND_TEMPLATE = 'arduino-cli core install {core}'
ARDUINO_LIBRARY_INSTALL_COMMAND_TEMPLATE = 'arduino-cli lib install {library}'
ARDUINO_COMPILE_COMMAND_TEMPLATE = 'arduino-cli compile -b {fqbn} "{sketch_path}"'
ARDUINO_UPLOAD_COMMAND_TEMPLATE = 'arduino-cli upload -b {fqbn} -p {port} "{sketch_path}"'

# Prefix for all output displayed by this script (not including subcommands)
OUTPUT_PREFIX = 'Arduino.py'

# Load arduino YAML file
try:
    config_file_resolved_path = rr.get_filename(CONFIG_YAML_PATH, use_protocol=False)
    with open(config_file_resolved_path) as f:
        config_data = yaml.safe_load(f)
        ARDUINO_DATA = config_data["arduino"]
except Exception:
    print(f'{OUTPUT_PREFIX}: FATAL ERROR: Could not load Arduino YAML file at {config_file_resolved_path}. '
          f'Please check if the file exists and is in valid YAML format.')
    exit(1)

# ======================================================================================================================
# Helper functions


def run_command(command: Union[str, Sequence[str]], print_output: bool, env_updates: Optional[dict] = None,
                path_to_run_at: str = None) -> None:
    """
    Run a command at a given path.

    Args:
        command: Command to run.
        print_output: Whether to allow the command to print to stdout.
        env_updates: Dictionary of environment variables to update. If None, no environment variables will be updated.
        path_to_run_at: Path to run the command at. If None, the command will be run at the current working directory.

    Raises:
        ValueError: If command empty.
        subprocess.CalledProcessError: If command returns non-zero exit code.
    """
    if not command:
        raise ValueError('Command must not be empty')

    env = os.environ.copy()
    if env_updates:
        for key, value in env_updates.items():
            env[key] = str(value)

    print(f'{OUTPUT_PREFIX}: CMD: {command}')
    subprocess.run(command, shell=True, check=True, stdout=subprocess.DEVNULL if not print_output else None,
                   stderr=subprocess.STDOUT, env=env, cwd=path_to_run_at)


def run_commands(commands: Sequence[str], print_output: bool, env_updates: Optional[dict] = None,
                 path_to_run_at: str = None) -> None:
    """
    Run a sequence of commands at a given path.

    Args:
        commands: Sequence of commands to run.
        print_output: Whether to allow the commands to print to stdout.
        env_updates: Dictionary of environment variables to update. If None, no environment variables will be updated.
        path_to_run_at: Path to run the commands at. If None, the commands will be run at the current working directory.

    Raises:
        ValueError: If commands empty.
        subprocess.CalledProcessError: If command returns non-zero exit code.
    """
    if not commands:
        raise ValueError('Commands must not be empty')

    for command in commands:
        run_command(command, print_output, env_updates=env_updates, path_to_run_at=path_to_run_at)


def get_arduino_cores(arduino_names: List[str]) -> List[str]:
    """
    Get the list of Arduino cores of the given Arduino names without duplicates.

    Args:
        arduino_names: List of Arduino names.

    Returns:
        List of Arduino cores.
    """
    cores = set()
    for arduino in arduino_names:
        cores.add(ARDUINO_DATA[arduino]['core'])

    return list(cores)


def get_arduino_libs(arduino_names: List[str]) -> List[str]:
    """
    Get the list of Arduino libraries of the given Arduino names without duplicates.

    Args:
        arduino_names: List of Arduino names.

    Returns:
        List of Arduino libraries.
    """
    libs = set()
    for arduino in arduino_names:
        if 'libraries' in ARDUINO_DATA[arduino] and ARDUINO_DATA[arduino]['libraries']:
            libs.update(ARDUINO_DATA[arduino]['libraries'])

    return list(libs)


def check_if_ros_lib_is_required(arduino_names: List[str]) -> bool:
    """
    Check if the ros_lib library is required for the given Arduino names.

    Args:
        arduino_names: List of Arduino names.

    Returns:
        True if ros_lib is required, False otherwise.
    """
    for arduino in arduino_names:
        if ARDUINO_DATA[arduino]['requires_ros_lib']:
            return True
    return False


def get_arduino_port(arduino_name: str) -> str:
    """
    Get the port of the requested Arduino.

    Args:
        arduino_name: Name of the Arduino.

    Returns:
        Port of the requested Arduino.

    Raises:
        StopIteration: If the port is not found.
    """
    ftdi_string = str(ARDUINO_DATA[arduino_name]['ftdi'])
    return next(list_ports.grep(ftdi_string)).device


def get_arduino_sketch_path_absolute(arduino_name: str) -> str:
    """
    Get the absolute path of the Arduino sketch.

    Args:
        arduino_name: Name of the Arduino.

    Returns:
        Absolute path of the Arduino sketch.
    """
    sketch_path_relative = ARDUINO_DATA[arduino_name]['sketch']
    return rr.get_filename(OFFBOARD_COMMS_PATH_TEMPLATE.format(subpath=sketch_path_relative), use_protocol=False)


# ======================================================================================================================
# Subparser functions


def install_libs(arduino_names: List[str], print_output: bool) -> None:
    """
    Install the required Arduino core libraries and, if required, ROS library, for the given Arduino names.

    Args:
        arduino_names: List of Arduino names.
        print_output: Whether to allow the commands to print to stdout.
    """

    # Get the list of unique Arduino cores that need to be installed and the list of commands to install the cores
    arduino_cores = get_arduino_cores(arduino_names)
    core_install_commands = [ARDUINO_CORE_INSTALL_COMMAND_TEMPLATE.format(core=core) for core in arduino_cores]

    # Get the list of unique Arduino libraries that need to be installed and the list of commands to install the
    # libraries
    arduino_libs = get_arduino_libs(arduino_names)
    lib_install_commands = [ARDUINO_LIBRARY_INSTALL_COMMAND_TEMPLATE.format(library=lib) for lib in arduino_libs]

    # Install the ros_lib library for Arduino if required
    if check_if_ros_lib_is_required(arduino_names):
        print(f'{OUTPUT_PREFIX}: Installing ros_lib library...')
        run_commands(ROS_LIB_INSTALL_COMMANDS, print_output, env_updates=ROS_LIB_ENV_UPDATES,
                     path_to_run_at=ROS_LIB_INSTALL_PATH)
        print(f'{OUTPUT_PREFIX}: ros_lib library installed.')
    else:
        print(f'{OUTPUT_PREFIX}: Skipped installing ros_lib library because it is not required for '
              f'{", ".join(arduino_names)} arduino(s).')

    # Print a linebreak
    print()

    # Install the Arduino cores
    if core_install_commands:
        print(f'{OUTPUT_PREFIX}: Installing arduino cores...')
        run_commands(core_install_commands, print_output)
        print(f'{OUTPUT_PREFIX}: Arduino cores installed.')
    else:
        print(f'{OUTPUT_PREFIX}: Skipped installing arduino cores because they are not required for '
              f'{", ".join(arduino_names)} arduino(s).')

    # Print a linebreak
    print()

    # Install the Arduino libraries
    if lib_install_commands:
        print(f'{OUTPUT_PREFIX}: Installing arduino libraries...')
        run_commands(lib_install_commands, print_output)
        print(f'{OUTPUT_PREFIX}: Arduino libraries installed.')
    else:
        print(f'{OUTPUT_PREFIX}: Skipped installing arduino libraries because they are not required for '
              f'{", ".join(arduino_names)} arduino(s).')


def find_ports(arduino_names: List[str], no_linebreaks: bool) -> None:
    """
    Find and print the port(s) of the requested Arduino names.

    Args:
        arduino_names: List of Arduino names.
        no_linebreaks: Whether to print the ports without labels or linebreaks.
    """
    # Get the port for each arduino
    for arduino_name in arduino_names:
        try:
            port = get_arduino_port(arduino_name)

            # If no_linebreaks is True, print the port without labels or linebreaks
            if no_linebreaks:
                print(port, end='')
            else:
                print(f'{OUTPUT_PREFIX}: {arduino_name.capitalize()}: {port}')
        except StopIteration:
            print(f'{OUTPUT_PREFIX}: Could not find port of {arduino_name} arduino.')


def compile(arduino_names: List[str], print_output: bool) -> None:
    """
    Install required libraries and compile the sketches for the given Arduino names.

    Args:
        arduino_names: List of Arduino names.
        print_output: Whether to allow the commands to print to stdout.
    """

    # Install the required libraries for the given Arduino names
    install_libs(arduino_names, print_output)

    # Print a linebreak
    print()

    # Compile the sketches for the given Arduino names
    for index, arduino_name in enumerate(arduino_names):
        print(f'{OUTPUT_PREFIX}: Compiling {arduino_name} arduino sketch...')

        sketch_path_absolute = get_arduino_sketch_path_absolute(arduino_name)
        fqbn = ARDUINO_DATA[arduino_name]['fqbn']

        # Run pre-compile command, if it exists
        if ARDUINO_DATA[arduino_name].get('pre_compile'):
            run_command(ARDUINO_DATA[arduino_name]['pre_compile'], print_output)

        # Compile the sketch
        run_command(ARDUINO_COMPILE_COMMAND_TEMPLATE.format(fqbn=fqbn, sketch_path=sketch_path_absolute), print_output)

        # Run post-compile command, if it exists
        if ARDUINO_DATA[arduino_name].get('post_compile'):
            run_command(ARDUINO_DATA[arduino_name]['post_compile'], print_output)

        print(f'{OUTPUT_PREFIX}: Compilation of {arduino_name} arduino sketch complete.')

        # Print a linebreak between arduino compilations
        if index < len(arduino_names) - 1:
            print()


def upload(arduino_names: List[str], print_output: bool) -> None:
    """
    Install required libraries, compile the sketches, and upload them to the given Arduino names.

    Args:
        arduino_names: List of Arduino names.
        print_output: Whether to allow the commands to print to stdout.
    """

    # Compile the sketches for the given Arduino names
    compile(arduino_names, print_output)

    # Print a linebreak
    print()

    print(f'{OUTPUT_PREFIX}: Finding ports of the given arduinos...')

    # Get the list of ports for the given Arduino names
    ports = {}
    error = False
    for arduino_name in arduino_names:
        try:
            ports[arduino_name] = get_arduino_port(arduino_name)
            print(f'{OUTPUT_PREFIX}: {arduino_name.capitalize()}: {ports[arduino_name]}')
        except StopIteration:
            error = True
            print(f'{OUTPUT_PREFIX}: FATAL ERROR: Could not find port of {arduino_name} arduino.')

    if error:
        print(f'{OUTPUT_PREFIX}: Could not upload sketches to arduinos due to errors in finding ports '
              'of given arduinos.')
        return

    print(f'{OUTPUT_PREFIX}: Given arduino ports found.')

    # Print a linebreak
    print()

    # Upload the sketch to the given Arduino names
    for index, arduino_name in enumerate(arduino_names):
        print(f'{OUTPUT_PREFIX}: Uploading to {arduino_name} arduino...')

        sketch_path_absolute = get_arduino_sketch_path_absolute(arduino_name)
        fqbn = ARDUINO_DATA[arduino_name]['fqbn']
        port = ports[arduino_name]

        # Upload the sketch
        run_command(ARDUINO_UPLOAD_COMMAND_TEMPLATE.format(fqbn=fqbn, port=port, sketch_path=sketch_path_absolute),
                    print_output)

        print(f'{OUTPUT_PREFIX}: Upload to {arduino_name} arduino complete.')

        # Print a linebreak between arduino uploads
        if index < len(arduino_names) - 1:
            print()


# ======================================================================================================================
# Argument parsing


if __name__ == "__main__":

    # Get the list of arduino names
    arduino_names = list(ARDUINO_DATA.keys())

    # Add 'all' to the list of arduino names to pass to argparse
    arduino_names_with_all = arduino_names + ['all']

    # Create argparse parser
    parser = argparse.ArgumentParser(description='CLI for installing libraries, finding ports, compiling, and '
                                                 'uploading sketches to Arduinos.')

    # Subparsers for different commands
    subparsers = parser.add_subparsers(dest='command', help='Available commands')

    # Subparser for install-libs command
    install_libs_parser = subparsers.add_parser('install-libs', help='Install core libraries and ROS library (if '
                                                                     'required) for Arduinos.')
    install_libs_parser.add_argument('arduino_names', nargs='+', choices=arduino_names_with_all,
                                     metavar='Arduino Names', help='Names of Arduinos to install libraries for. Use '
                                                                   '"all" to install libraries for all Arduinos.')
    install_libs_parser.add_argument('-p', '--print-output', action='store_true', help='Print output of subcommands.')

    # Subparser for find-ports command
    find_ports_parser = subparsers.add_parser('find-ports', help='Find ports of Arduinos.')
    find_ports_parser.add_argument('arduino_names', nargs='+', choices=arduino_names_with_all, metavar='Arduino Names',
                                   help='Names of Arduinos whose ports to find. Use "all" to find ports of all'
                                        'Arduinos.')
    find_ports_parser.add_argument('--nl', '--no-linebreaks', action='store_true',
                                   help='Print ports without labels or linebreaks.')

    # Subparser for compile command
    compile_parser = subparsers.add_parser('compile', help='Install required libraries and compile Arduino sketches.')
    compile_parser.add_argument('arduino_names', nargs='+', choices=arduino_names_with_all, metavar='Arduino Names',
                                help='Names of Arduinos whose sketches to compile. Use "all" to compile sketches of '
                                     'all Arduinos.')
    compile_parser.add_argument('-p', '--print-output', action='store_true', help='Print output of subcommands.')

    # Subparser for upload command
    upload_parser = subparsers.add_parser('upload', help='Install required libraries, compile, and upload sketches '
                                                         'to Arduinos.')
    upload_parser.add_argument('arduino_names', nargs='+', choices=arduino_names_with_all, metavar='Arduino Names',
                               help='Names of Arduinos whose sketches to compile and upload. Use "all" to upload '
                                    'sketches of all Arduinos.')
    upload_parser.add_argument('-p', '--print-output', action='store_true', help='Print output subcommands.')

    # Parse command line arguments
    args = parser.parse_args()

    # Replace 'all' with all actual arduino names
    if 'all' in args.arduino_names:
        args.arduino_names = arduino_names

    # Remove duplicates
    else:
        args.arduino_names = list(set(args.arduino_names))

    # Perform actions based on the command
    if args.command == 'install-libs':
        install_libs(args.arduino_names, args.print_output)
    elif args.command == 'find-ports':
        find_ports(args.arduino_names, args.nl)
    elif args.command == 'compile':
        compile(args.arduino_names, args.print_output)
    elif args.command == 'upload':
        upload(args.arduino_names, args.print_output)
    else:
        print('Invalid command.')
        parser.print_help()
