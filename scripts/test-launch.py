#!/usr/bin/env python

import roslaunch
import rospy
import os

# http://wiki.ros.org/roslaunch/API%20Usage
# https://stackoverflow.com/questions/2212643/python-recursive-folder-read

basedir = '../onboard/catkin_ws/src/'

launchfiles = []
for dirpath, dirnames, filenames in os.walk(basedir):
    for filename in filenames:
        if filename.endswith('.launch'):
            launchfile_path = os.path.join(dirpath, filename)
            launchfiles.append(launchfile_path)

print("Found the following launchfiles in {0}:\n{1}".format(basedir, launchfiles))

# rospy.init_node('launch-tester', anonymous=True)
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)

launch = roslaunch.parent.ROSLaunchParent(uuid, launchfiles)
launch.start()
rospy.loginfo("started")

rospy.sleep(3)  # wait 3 seconds to try and generate an error
launch.shutdown()
