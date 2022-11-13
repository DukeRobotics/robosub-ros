#!/usr/bin/env python3

import os
import sys
import psutil
import subprocess
import signal

BLOCK_LIST = [
    'pub_joy.launch.py',
    'gui.launch.py',  # TODO: Remove this once gui is migrated properly
    'base_sim.launch.py',
    'test_sim_comm.launch.py',
]


class bcolors:
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'


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
    for package in launchfiles:
        for launchfile in launchfiles[package]:
            print(bcolors.WARNING + f"Launching {launchfile}." + bcolors.ENDC)
            proc = subprocess.Popen(['ros2', 'launch', package, launchfile])

            try:
                proc.communicate(input=b'\n', timeout=timeout)
                print(bcolors.FAIL + f"FAIL {launchfile}. ROS crashed during launch." + bcolors.ENDC)
                sys.exit(1)
            except subprocess.TimeoutExpired:
                # If timeout expires, process is still up
                print(bcolors.OKGREEN + f"PASS {launchfile}." + bcolors.ENDC)

            try:
                proc.send_signal(signal.SIGINT)
                proc.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                print(bcolors.FAIL + "Failed to shut launch down" + bcolors.ENDC)
                sys.exit(1)

    print(bcolors.OKGREEN + "ALL PASSED." + bcolors.ENDC)


if __name__ == '__main__':
    test_launch()
