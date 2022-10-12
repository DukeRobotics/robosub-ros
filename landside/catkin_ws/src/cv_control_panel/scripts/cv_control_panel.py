#!/usr/bin/env python3

# Run thse commands after running build.sh and sourcing setup.bash
# You only need to do this once after a docker-compose
# source /opt/ros/noetic/setup_network.bash
# apt-get update
# apt-get install python3-tk

# To run this file, run the roslaunch below
# roslaunch cv_control_panel cv_control_panel.launch

# If you get an error saying this file is not executable
# chmod +x path_to_this_file
# From /dev/robosub-ros/: chmod +x landside/catkin_ws/src/cv_control_panel.py

from tkinter import *
  
root = Tk()
a = Label(root, text ="Hello World")
a.pack()
  
root.mainloop()