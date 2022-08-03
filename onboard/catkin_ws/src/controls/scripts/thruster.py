#!/usr/bin/env python3

import numpy as np
from tf.transformations import quaternion_from_euler
from controls_utils import quat_vec_mult


class Thruster:
    """Describes a thruster in terms of position and orientation. Calculates the
    force and torque that the thruster applies when spun.
    
    Attributes:
        flipped: Whether the thruster is reversed, meaning power has to be negated
        force_hat: The unit force vector (N) applied by the thruster in x, y, z 
        pos: The x, y, z position of the thruster (m) relative to the robot's center of mass
        rpy: The euler angle orientation of the thruster (degrees) relative to the robot's frame of reference
        torque: The torque vector (N m) applied by the thruster
    """
    
    def __init__(self, pos, rpy, flipped):
        self.pos = pos
        self.rpy = rpy
        self.flipped = flipped

        # create a quaternion from euler rpy
        q = quaternion_from_euler(np.deg2rad(rpy[0]), np.deg2rad(rpy[1]), np.deg2rad(rpy[2]), 'sxyz')
        # sxyz denotes a static (extrinsic) frame of reference.
        # https://github.com/matthew-brett/transforms3d/blob/master/transforms3d/euler.py
        # https://en.wikipedia.org/wiki/Euler_angles

        self.force_hat = quat_vec_mult(q, [1, 0, 0])  # unit vector representing force of thruster in x,y,z directions
        self.torque = np.cross(self.pos, self.force_hat)  # vector representing torque generated from thruster