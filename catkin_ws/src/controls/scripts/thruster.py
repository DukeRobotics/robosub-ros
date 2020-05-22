#!/usr/bin/env python

import rospy
import numpy as np
from tf.transformations import quaternion_from_euler
import math
from drc_math import quat_vec_mult 

class Thruster():

    # Constants for scaling thruster speeds to ROS thruster controller
    MAX_NEG_POW = -128
    MAX_POS_POW = 127

    def __init__(self, pos, rpy, flipped):
        self.pos = pos
        self.rpy = rpy
        self.flipped = flipped
        
        to_rad = lambda x: x * math.pi / 180  # convert degrees to radians

        q = quaternion_from_euler(to_rad(rpy[0]), to_rad(rpy[1]), to_rad(rpy[2]), 'sxyz') # create a quaternion from euler rpy
        # sxyz denotes a static (extrinsic) frame of reference.
        # https://github.com/matthew-brett/transforms3d/blob/master/transforms3d/euler.py
        # https://en.wikipedia.org/wiki/Euler_angles

        self.force_hat = quat_vec_mult(q, [1, 0, 0]) # vector representing force of thruster in x,y,z directions
        
        self.torque = np.cross(self.pos, self.force_hat) # vector representing torque generated from thruster
