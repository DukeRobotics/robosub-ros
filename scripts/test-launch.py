#!/usr/bin/env python3

import roslaunch
import rospy
import os
import sys
import psutil
import subprocess

IGNORE_LIST = []
BLOCK_LIST = [
    'cv/launch/cv.launch',
    'cv/launch/depthai_service_model.launch',
    'cv/launch/usb_camera.launch',
    'joystick/launch/pub_joy.launch',
    'simulation/launch/base_sim.launch',
    'simulation/launch/test_sim_comm.launch',
    'sonar/launch/sonar.launch',
    'task_planning/launch/task_runner.launch',
    'vectornav/launch/vectornav.launch',
    'execute/launch/tasks.launch'
]


class bcolors:
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'


def get_ws_file(abs_path):
    return abs_path.split('catkin_ws/src/')[1]


def check_roslaunch_up():
    for proc in psutil.process_iter(['pid', 'name']):
        if proc.info['name'] == 'rosmaster':
            return True
    return False


def get_launchfiles(basedir):
    launchfiles = []
    for dirpath, dirnames, filenames in os.walk(basedir):
        for filename in filenames:
            if not filename.endswith('.launch'):
                continue
            filepath = os.path.join(dirpath, filename)
            if get_ws_file(filepath) not in BLOCK_LIST:
                launchfiles.append([filepath, 'sim:=true'])
    return launchfiles


def test_launch():

    timeout = 10

    ws = os.environ['COMPUTER_TYPE']

    if ws == 'landside':
        del os.environ['ROS_MASTER_URI']
        _ = subprocess.Popen(['Xvfb', ':99'])
        os.environ["DISPLAY"] = ":99"

    launchfiles = get_launchfiles(f'{ws}/catkin_ws/src/')
    print(len(launchfiles))
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    if check_roslaunch_up():
        print("rosmaster is already running, please stop before running this test")
        sys.exit(1)

    for launchfile in launchfiles:
        if check_roslaunch_up():
            print("Failed to shut launch down")
            sys.exit(1)
        launch = roslaunch.parent.ROSLaunchParent(
            uuid, [(roslaunch.rlutil.resolve_launch_arguments(launchfile)[0], launchfile[1:])], force_required=True)
        print(f"Launching launchfile: {launchfile[0]}.")
        launch.start()
        rospy.sleep(timeout)
        if get_ws_file(launchfile[0]) in IGNORE_LIST and not check_roslaunch_up():
            print(bcolors.WARNING + f"IGNORE {launchfile[0]}. Ignoring error due to IGNORE_LIST." + bcolors.ENDC)
        elif not check_roslaunch_up():
            print(bcolors.FAIL + f"FAIL {launchfile[0]}. ROS crashed during launch." + bcolors.ENDC)
            sys.exit(1)
        else:
            print(bcolors.OKGREEN + f"PASS {launchfile[0]}." + bcolors.ENDC)
        launch.shutdown()
    print(bcolors.OKGREEN + "ALL PASSED." + bcolors.ENDC)


if __name__ == '__main__':
    test_launch()
