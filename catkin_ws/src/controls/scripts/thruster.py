#!/usr/bin/env python

import rospy
import numpy as np
from tf.transformations import quaternion_from_euler
import math
from drc_math import quad_vec_mult 

class Thruster():

    # Constants for scaling thruster speeds to ROS thruster controller
    MAX_NEG_POW = -128
    MAX_POS_POW = 127

    def __init__(self, pos, rpy):
        self.pos = pos
        self.rpy = rpy
        
        to_rad = lambda x: x * math.pi / 180  # convert degrees to radians
        q = quaternion_from_euler(to_rad(rpy[0]), to_rad(rpy[1]), to_rad(rpy[2])) # create a quaternion from euler rpy

        self.force_hat = quad_vec_mult(q, [1, 0, 0]) # vector representing force of thruster in x,y,z directions
        
        self.torque = np.cross(self.pos, self.force_hat) # vector representing torque generated from thruster
