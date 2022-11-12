import os
import glob

import xml.etree.ElementTree as ET 

class GetLaunchArgs:
    
    # TODO: Complete this function. Currently it prints all tags and attributes in the launch file
    # Given a path to a launch file, get the args accepted by that launch file and their default values
    # Do not args with a specified value (i.e. value="some_value")
    def launch_args(self, launch_file_path):
        tree = ET.parse(launch_file_path)

        def print_recursively(root):
            for child in root:
                print(child.tag, child.attrib)
                print_recursively(child)
        
        root = tree.getroot()
        print_recursively(root)
        print()

cv_launch_dir = "PATH_TO_ROBOSUB_ROS/robosub-ros/onboard/catkin_ws/src/cv/launch/"
gla = GetLaunchArgs()
gla.launch_args(cv_launch_dir + "cv.launch")
gla.launch_args(cv_launch_dir + "test_images.launch")
