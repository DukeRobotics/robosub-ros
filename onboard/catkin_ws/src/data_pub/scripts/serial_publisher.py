#!/usr/bin/env python3

from abc import ABC, abstractmethod
import rospy
import os
import time
import serial
import serial.tools.list_ports as list_ports
import yaml
import resource_retriever as rr
import traceback
from std_msgs.msg import Float64

# Used for sensor fusion
from geometry_msgs.msg import PoseWithCovarianceStamped


class SerialPublisher(ABC):


    def __init__(self, *, node_name, baud, config_file_path):

        self.config_file_path = config_file_path
        self.baud = baud
        self.node_name = node_name

        with open(rr.get_filename(self.config_file_path, use_protocol=False)) as f:
            config_data = yaml.safe_load(f)
            self._arduino_config = config_data['arduino']

