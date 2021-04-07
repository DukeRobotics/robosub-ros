#!/usr/bin/env python

import roslaunch
import rospy
import os
import sys

# http://wiki.ros.org/roslaunch/API%20Usage
# https://stackoverflow.com/questions/2212643/python-recursive-folder-read

if len(sys.argv) < 2 or (sys.argv[1] != "onboard" and sys.argv[1] != "landside"):
    exit("Invalid arguments. Usage: test-launch.py <onboard/landside> [timeout]")
if len(sys.argv) > 2:
    timeout = int(sys.argv[2])
else:
    timeout = 3

# launch-test-deny-list.txt contains the relative filepath of any file that we
# should skip for the launching test, with one filepath per line
deny_list = []
denylist_file = open("launch-test-deny-list.txt", "r")
for denied in denylist_file:
    deny_list.append(denied)

basedir = '../{0}/catkin_ws/src/'.format(sys.argv[1])

launchfiles = []
for dirpath, dirnames, filenames in os.walk(basedir):
    for filename in filenames:
        if filename.endswith('.launch'):
            launchfile_path = os.path.join(dirpath, filename)
            launchfiles.append([launchfile_path])

tested_launchfiles = [p for p in launchfiles if p[0] not in deny_list]

print("About to test the following launchfiles in {0}:\n{1}".format(basedir, tested_launchfiles))

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)

for launchfile in tested_launchfiles:
    try:
        launch = roslaunch.parent.ROSLaunchParent(uuid, launchfile)
        launch.start()
        msg = "Launching launchfile: {0}.".format(launchfile[0])
        rospy.loginfo(msg)
        print(msg)

        rospy.sleep(timeout)  # wait 3 seconds to try and generate an error=
        launch.shutdown()
        print("Successfully launched {0}.".format(launchfile[0]))
    except roslaunch.RLException as exception:
        print("Failed launch for {0}.".format(launchfile[0]))
        raise exception
