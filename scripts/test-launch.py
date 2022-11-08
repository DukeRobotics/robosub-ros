#!/usr/bin/env python3

import os
import sys
import psutil
import subprocess
import time

import signal

IGNORE_LIST = []
BLOCK_LIST = [
    'cv.launch.py',
    'pub_joy.launch.py',
    'base_sim.launch.py',
    'test_sim_comm.launch.py',
]


class bcolors:
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'


def get_ws_file(abs_path):
    return abs_path.split('ros2_ws/src/')[1]


def check_roslaunch_up():
    """ Checks if any ros2 processes are running """
    for proc in psutil.process_iter(['pid', 'name']):
        if proc.info['name'] == 'ros2':
            return True
    return False


def get_launchfiles(basedir):
    """ Return map of package name to launch file """
    launchfiles = {}
    for dirpath, dirnames, filenames in os.walk(basedir):
        for filename in filenames:
            if not filename.endswith('.launch.py'):
                continue
            if filename not in BLOCK_LIST:
                package = dirpath.split('/')[-2]
                if package not in launchfiles:
                    launchfiles[package] = []
                launchfiles[package].append(filename)
    return launchfiles


def test_launch():

    timeout = 10
    ws = os.environ['COMPUTER_TYPE']

    if ws == 'landside':
        _ = subprocess.Popen(['Xvfb', ':99'])
        os.environ["DISPLAY"] = ":99"

    launchfiles = get_launchfiles(f'{ws}/ros2_ws/src/')

    if check_roslaunch_up():
        print("ROS is already running, please stop before running this test")
        sys.exit(1)

    for package in launchfiles:
        for launchfile in launchfiles[package]:

            print(f"Launching launchfile: {launchfile}.")
            proc = subprocess.Popen(['ros2', 'launch', package, launchfile])
            time.sleep(timeout)

            if not check_roslaunch_up():
                print(bcolors.FAIL + f"FAIL {launchfile}. ROS crashed during launch." + bcolors.ENDC)
                sys.exit(1)
            else:
                print(bcolors.OKGREEN + f"PASS {launchfile}." + bcolors.ENDC)

            proc.send_signal(signal.SIGINT)
            proc.wait(timeout=timeout)
            if check_roslaunch_up():
                print("Failed to shut launch down")
                sys.exit(1)
    print(bcolors.OKGREEN + "ALL PASSED." + bcolors.ENDC)


if __name__ == '__main__':
    test_launch()
